/**
  ******************************************************************************
  * @file    main.c
  * @brief   项目：HC-SR04 超声波测距报警器 (STM32F103C8T6 + 标准外设库 SPL)
  * @version v1.0
  *
  * 硬件接线
  *   HC-SR04  TRIG -> PB6   推挽输出（空闲低电平）
  *   HC-SR04  ECHO -> PB7   浮空输入，接 TIM4_CH2 做输入捕获
  *   OLED     SCL  -> PB8   I2C1 重映射（复用开漏）
  *   OLED     SDA  -> PB9   I2C1 重映射（复用开漏）  从机地址 0x78
  *   蜂鸣器   信号 -> PA0   定时器 PWM 输出（TIM2_CH1）
  *
  * 功能
  *   1. TIM4 工作在 1MHz，用输入捕获测量 ECHO 高电平脉宽，
  *      距离(cm) = 脉宽(us) / 58
  *   2. OLED 第 1 行固定显示报警阈值 10cm，中间显示实时距离
  *   3. 距离 < 10cm 时 PA0 输出 2.7kHz 方波驱动蜂鸣器；平时输出高电平（关断）
  *      （套件里的蜂鸣器模块是"低电平触发 + 无源"，必须给方波才响，
  *        而且通电条件是低电平，所以静音时输出高电平，详见第四节）
  *
  * 说明
  *   - 测距流程：主循环把 TRIG 拉高约 10us -> HC-SR04 发 8 个 40kHz 脉冲
  *     -> ECHO 变高。TIM4_CH2 先捕获上升沿，捕获到以后自动改成捕获下降沿，
  *     下降沿捕获值减去上升沿捕获值就是回波高电平的微秒数。
  *   - 距离用整数算，精度 0.1cm：distCm10 = us * 10 / 58
  *   - 超量程 / 没插传感器时距离显示 "--.-"，此时蜂鸣器保持不响
  ******************************************************************************
  */

#include "stm32f10x.h"   /* 本工程已把全部 SPL 驱动头文件包含进来 */
#include "delay.h"       /* Delay / DelayUs / GetTick */
#include "i2c.h"         /* 硬件 I2C（My_I2C_SendBytes） */
#include "oled.h"        /* OLED 驱动（OLED_SLAVE_ADDR 就是 0x78） */
#include <stdio.h>

/* ==========================================================================
 *                        一、硬件配置区（改接线只改这里）
 * ========================================================================== */

/* ---------- HC-SR04 ---------- */
#define TRIG_GPIO            GPIOB
#define TRIG_PIN             GPIO_Pin_6
#define ECHO_GPIO            GPIOB
#define ECHO_PIN             GPIO_Pin_7      /* 同时是 TIM4_CH2 的输入脚 */

/* ---------- 蜂鸣器：TIM2_CH1 = PA0 ---------- */
#define BUZZER_TIM           TIM2
#define BUZZER_GPIO          GPIOA
#define BUZZER_PIN           GPIO_Pin_0
#define BUZZER_TONE_HZ       2700            /* 无源蜂鸣器的发声频率 */

/* ---------- OLED：硬件 I2C1（重映射后 SCL=PB8、SDA=PB9） ---------- */
#define OLED_I2C             I2C1
#define OLED_SCL_PIN         GPIO_Pin_8
#define OLED_SDA_PIN         GPIO_Pin_9
#define OLED_I2C_SPEED       400000          /* SSD1306 支持 400kHz */

/* ---------- 测距参数 ---------- */
#define ALARM_DIST_CM        10              /* 报警阈值，就是第一行显示的那个 10cm */
#define US_PER_CM            58              /* 声速换算：1cm 约 58us */
#define ECHO_MIN_US          100             /* 小于 100us 视为无效（约 1.7cm） */
#define ECHO_MAX_US          24000           /* 大于 24000us 视为超量程（约 413cm） */
#define MEAS_PERIOD_MS       100             /* 测距周期 */
#define ECHO_TIMEOUT_MS      60              /* 等回波超时（无回波时 ECHO 约 38ms 才拉低） */
#define UI_PERIOD_MS         100             /* 屏幕刷新周期 */
#define DIST_UNKNOWN         0xFFFFFFFFUL    /* 无效距离 */

/* ==========================================================================
 *                        二、全局变量
 * ========================================================================== */

/* ---- 输入捕获状态，由 TIM4_IRQHandler（stm32f10x_it.c）更新 ---- */
volatile uint8_t  g_capLevel = 0;      /* 0 = 等上升沿，1 = 等下降沿 */
volatile uint16_t g_capRise  = 0;      /* 上升沿的捕获值 */
volatile uint16_t g_echoUs   = 0;      /* 回波高电平脉宽，单位 us */
volatile uint8_t  g_echoDone = 0;      /* 1 = 本次回波测量完成 */

/* ---- 本文件内部使用 ---- */
static OLED_TypeDef g_oled;
static uint8_t      g_oledOk    = 0;             /* 1 = 屏幕初始化成功 */

static uint32_t     g_distCm10  = DIST_UNKNOWN;  /* 距离，单位 0.1cm */
static uint8_t      g_alarm     = 0;             /* 1 = 正在报警 */

static uint8_t      g_measBusy  = 0;             /* 1 = 正在等这次的回波 */
static uint32_t     g_trigTick  = 0;             /* 本次发触发脉冲的时刻 */
static uint32_t     g_lastTrig  = 0;             /* 上次发触发脉冲的时刻 */
static uint32_t     g_aliveCnt  = 0;             /* 主循环计数，用 SWD 看程序是否在跑 */

/* ==========================================================================
 *                        三、TRIG（PB6）和 蜂鸣器（PA0）
 * ========================================================================== */

static void Trig_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* PB6 TRIG：推挽输出，空闲保持低电平 */
    gpio.GPIO_Pin   = TRIG_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(TRIG_GPIO, &gpio);
    GPIO_ResetBits(TRIG_GPIO, TRIG_PIN);
}

/* 发一次触发脉冲：拉高约 10us 再拉低 */
static void Ultrasonic_Trigger(void)
{
    GPIO_SetBits(TRIG_GPIO, TRIG_PIN);
    DelayUs(10);
    GPIO_ResetBits(TRIG_GPIO, TRIG_PIN);
}

/* --------------------------------------------------------------------------
 *  蜂鸣器（PA0 = TIM2_CH1）
 *
 *  套件里这个模块是"低电平触发 + 无源"的：
 *    - 无源       ：必须给它方波才发声，给固定的高/低电平只会"咔"一声；
 *    - 低电平触发 ：给它低电平才通电，和"高电平响"正好相反。
 *  所以这里用 TIM2_CH1 输出 2.7kHz 方波：
 *    距离 < 10cm ：输出 50% 方波  -> 蜂鸣器响
 *    平时        ：比较值给 0，PWM 模式2 输出恒高电平 -> 模块关断、安静
 *
 *  如果以后换成"高电平触发"的有源蜂鸣器，把下面的 PWM 模式改成
 *  TIM_OCMode_PWM1、静音时的比较值改成 ARR+1 即可，逻辑正好相反。
 * ------------------------------------------------------------------------ */

static uint16_t g_buzzerArr = 0;

static void Buzzer_Init(void)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef       oc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* PA0 = TIM2_CH1（不需要重映射），复用推挽输出 */
    gpio.GPIO_Pin   = BUZZER_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BUZZER_GPIO, &gpio);

    /* 72MHz / 72 = 1MHz，1 个计数 = 1us */
    tim.TIM_Prescaler         = 72 - 1;
    tim.TIM_CounterMode       = TIM_CounterMode_Up;
    tim.TIM_Period            = (uint16_t)(1000000UL / BUZZER_TONE_HZ - 1);
    tim.TIM_ClockDivision     = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(BUZZER_TIM, &tim);

    g_buzzerArr = tim.TIM_Period;

    oc.TIM_OCMode       = TIM_OCMode_PWM2;
    oc.TIM_OutputState  = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputNState_Disable;
    oc.TIM_Pulse        = 0;                 /* 上电先安静（恒高电平） */
    oc.TIM_OCPolarity   = TIM_OCPolarity_High;
    oc.TIM_OCNPolarity  = TIM_OCNPolarity_High;
    oc.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC1Init(BUZZER_TIM, &oc);
    TIM_OC1PreloadConfig(BUZZER_TIM, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(BUZZER_TIM, ENABLE);

    TIM_SetCompare1(BUZZER_TIM, 0);
    TIM_Cmd(BUZZER_TIM, ENABLE);
}

/* alarm = 1：输出 50% 方波，蜂鸣器响；alarm = 0：输出恒高电平，安静 */
static void Buzzer_SetAlarm(uint8_t alarm)
{
    if (alarm)
    {
        TIM_SetCompare1(BUZZER_TIM, (uint16_t)(g_buzzerArr / 2 + 1));
    }
    else
    {
        TIM_SetCompare1(BUZZER_TIM, 0);
    }
}

/* ==========================================================================
 *        四、TIM4_CH2 输入捕获（PB7）：把回波脉宽测成微秒数
 *   定时器时钟 72MHz / 72 = 1MHz，1 个计数就是 1us。
 * ========================================================================== */

static void Capture_Init(void)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tim;
    TIM_ICInitTypeDef       ic;
    NVIC_InitTypeDef        nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    /* PB7：输入捕获用的就是普通输入引脚 */
    gpio.GPIO_Pin  = ECHO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(ECHO_GPIO, &gpio);

    /* 1MHz 计数，ARR=0xFFFF 时一次最多量 65.5ms */
    tim.TIM_Prescaler         = 72 - 1;
    tim.TIM_CounterMode       = TIM_CounterMode_Up;
    tim.TIM_Period            = 0xFFFF;
    tim.TIM_ClockDivision     = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &tim);

    /* CH2 直连，先捕获上升沿 */
    ic.TIM_Channel     = TIM_Channel_2;
    ic.TIM_ICPolarity  = TIM_ICPolarity_Rising;
    ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
    ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    ic.TIM_ICFilter    = 0x0;
    TIM_ICInit(TIM4, &ic);

    TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
    TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);

    nvic.NVIC_IRQChannel                   = TIM4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM4, ENABLE);
}

/* 脉宽(us) -> 距离(0.1cm) */
static uint32_t Echo_ToDistCm10(uint16_t us)
{
    if (us < ECHO_MIN_US || us > ECHO_MAX_US) return DIST_UNKNOWN;

    return (uint32_t)us * 10 / US_PER_CM;
}

/* 每 MEAS_PERIOD_MS 测一次；一次测距分两步：发触发脉冲、等回波结果 */
static void Ultrasonic_Service(void)
{
    uint32_t now = GetTick();
    uint16_t us;

    if (g_measBusy == 0)
    {
        if ((uint32_t)(now - g_lastTrig) >= MEAS_PERIOD_MS)
        {
            g_lastTrig = now;
            g_trigTick = now;
            g_measBusy = 1;

            /* 先清完成标志、把捕获重新武装成等上升沿，再发脉冲，
             * 这样每次测量都从干净的状态开始，不会读到上一次的结果 */
            __disable_irq();
            g_echoDone = 0;
            g_capLevel = 0;
            TIM_OC2PolarityConfig(TIM4, TIM_ICPolarity_Rising);
            __enable_irq();

            Ultrasonic_Trigger();
        }
        return;
    }

    if (g_echoDone)
    {
        __disable_irq();              /* 这几个变量要么一起读，要么都不读 */
        us         = g_echoUs;
        g_echoDone = 0;
        __enable_irq();

        g_measBusy = 0;
        g_distCm10 = Echo_ToDistCm10(us);
    }
    else if ((uint32_t)(now - g_trigTick) >= ECHO_TIMEOUT_MS)
    {
        /* 一直没等到回波：重新武装成等上升沿，避免卡在下降沿 */
        __disable_irq();
        g_capLevel = 0;
        TIM_OC2PolarityConfig(TIM4, TIM_ICPolarity_Rising);
        __enable_irq();

        g_measBusy = 0;
        g_distCm10 = DIST_UNKNOWN;
    }
}

/* ==========================================================================
 *                        五、报警输出（PA0）
 * ========================================================================== */

static void Alarm_Service(void)
{
    g_alarm = ((g_distCm10 != DIST_UNKNOWN) && (g_distCm10 < (uint32_t)ALARM_DIST_CM * 10))
              ? 1 : 0;

    Buzzer_SetAlarm(g_alarm);
}

/* ==========================================================================
 *                        六、OLED：硬件 I2C1
 * ========================================================================== */

static int Oled_Write(uint8_t addr, const uint8_t *pdata, uint16_t size)
{
    return My_I2C_SendBytes(OLED_I2C, addr, pdata, size);
}

static void Oled_I2c_Init(void)
{
    GPIO_InitTypeDef gpio;
    I2C_InitTypeDef  i2c;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    /* I2C1 默认在 PB6/PB7（被 HC-SR04 占了），要用 PB8/PB9 必须重映射 */
    GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);

    gpio.GPIO_Pin   = OLED_SCL_PIN | OLED_SDA_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_AF_OD;      /* I2C 必须用复用开漏 */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    I2C_DeInit(OLED_I2C);

    i2c.I2C_Mode                = I2C_Mode_I2C;
    i2c.I2C_DutyCycle           = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1         = 0x00;
    i2c.I2C_Ack                 = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2c.I2C_ClockSpeed          = OLED_I2C_SPEED;
    I2C_Init(OLED_I2C, &i2c);

    I2C_Cmd(OLED_I2C, ENABLE);
}

/* 在 (x, yBase) 处画一行字符串，yBase 是字符底部的 y 坐标 */
static void Ui_Text(int16_t x, int16_t yBase, const char *str)
{
    OLED_SetPen(&g_oled, PEN_COLOR_WHITE, 1);
    OLED_SetBrush(&g_oled, BRUSH_TRANSPARENT);
    OLED_SetCursor(&g_oled, x, yBase);
    OLED_DrawString(&g_oled, str);
}

/* 整屏重画（默认字体 8 像素高、7 像素宽，一屏 4 行） */
static void Ui_Draw(void)
{
    char buf[24];

    OLED_Clear(&g_oled);

    /* 第 1 行：固定显示报警阈值 */
    Ui_Text(0, 15, "Threshold 10cm");

    /* 第 2、3 行（屏幕中间）：实时距离 */
    Ui_Text(0, 31, "Distance");

    if (g_distCm10 == DIST_UNKNOWN)
    {
        Ui_Text(28, 47, "--.- cm");
    }
    else
    {
        sprintf(buf, "%u.%u cm", (unsigned)(g_distCm10 / 10), (unsigned)(g_distCm10 % 10));
        Ui_Text(28, 47, buf);
    }

    /* 第 4 行：报警状态 */
    if (g_alarm)
    {
        Ui_Text(0, 63, "ALARM  <10cm");
    }
    else
    {
        Ui_Text(0, 63, "OK");
    }
}

/* ==========================================================================
 *                        七、main
 * ========================================================================== */

int main(void)
{
    OLED_InitTypeDef oledCfg;
    uint32_t         tUi;
    uint8_t          i;

    /* ---- 1. 外设初始化 ---- */
    Delay_Init();
    Trig_Init();          /* PB6 TRIG */
    Buzzer_Init();        /* PA0 蜂鸣器 PWM */
    Capture_Init();       /* TIM4_CH2 输入捕获 */
    Oled_I2c_Init();      /* 硬件 I2C1 */

    /* ---- 2. OLED 初始化 ----
     * SSD1306 上电后要等 100ms 以上才能应答 I2C，程序一上电就去找它必然失败
     * （现象就是冷启动黑屏、按一下复位又好了），所以先等 250ms 再重试 3 次。
     * 屏幕没接好也只是不显示，不影响测距和报警。 */
    Delay(250);

    oledCfg.i2c_write_cb = Oled_Write;

    for (i = 0; i < 3; i++)
    {
        if (OLED_Init(&g_oled, &oledCfg) == 0)
        {
            g_oledOk = 1;
            break;
        }
        Delay(50);
    }

    if (g_oledOk)
    {
        OLED_Clear(&g_oled);
        Ui_Text(0, 15, "HC-SR04 Ready");
        Ui_Text(0, 31, "STM32F103C8T6");
        OLED_SendBuffer(&g_oled);
    }

    Delay(500);

    /* ---- 3. 主循环 ---- */
    tUi = GetTick();

    while (1)
    {
        uint32_t now = GetTick();

        Ultrasonic_Service();
        Alarm_Service();

        if ((uint32_t)(now - tUi) >= UI_PERIOD_MS)
        {
            tUi = now;

            if (g_oledOk)
            {
                Ui_Draw();
                OLED_SendBuffer(&g_oled);
            }
        }

        g_aliveCnt++;
    }
}

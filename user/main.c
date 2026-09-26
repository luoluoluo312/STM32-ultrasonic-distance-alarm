/**
  ******************************************************************************
  * @file    main.c
  * @brief   项目一：光照监测与天黑报警器  (STM32F103C8T6 + 标准外设库 SPL)
  * @version v1.1   （本版：去掉 Flash 模块；按实机校准光敏方向）
  *
  * 功能一览
  *   1. 光敏传感器(ADC1_IN0 / PA0) + 电位器(ADC1_IN1 / PA1) 双通道采集，
  *      8 次均值 + 一阶低通滤波，数值稳定不跳动
  *      【光敏方向已按实机校准】光照越强 -> 界面"光照"数越大；天黑 -> 数变小
  *   2. 0.96 寸 I2C OLED 四页中文界面（16x16 点阵汉字）
  *        第1页 光照/阈值实时值 + 两条柱状条 + 电位器读数 + 报警开关
  *        第2页 亮度曲线（128 点，约 32 秒窗口）
  *        第3页 参数设置（K2/K3 加减阈值，在设置页长按 K4 恢复默认阈值）
  *        第4页 历史统计（本次上电后的条数/平均/最大/最小，K3 长按清空）
  *        报警时顶部出现"报警中"反白横幅
  *   3. 无源蜂鸣器(PWM, TIM4_CH1 / PB6)：报警节奏音 + 按键提示音
  *      （你的模块是"低电平触发"，用 BUZZER_ACTIVE_LOW 开关适配）
  *   4. 四个按键：翻页 / 阈值加减 / 报警开关 / 恢复默认阈值
  *   5. 历史统计放在 RAM 里（不用 Flash 模块；掉电后统计和阈值设置会复位）
  *   6. USART1(PA9/PA10) 每 200ms 上报一行 CSV，可用电脑串口助手观察
  *   7. 板载 LED(PC13)：报警时快闪，正常时慢闪
  *
  * 接线表见 my_app/wiring_table.md；所有引脚都在下面的"硬件配置区"里
  *
  * 注意：本文件用 #include "../my_app/cn_font_data.h" 引入中文字库，
  *       所以 main.c 必须和 my_app 文件夹放在同一个工程里（现在就是这样）。
  *       字库可以用 my_app/gen_cn_font.py 重新生成、增删汉字。
  *       界面预览见 my_app/ui_preview.png。
  ******************************************************************************
  */

#include "stm32f10x.h"   /* 本工程已把全部 SPL 驱动头文件包含进来 */
#include "delay.h"       /* Delay / GetTick / GetUs */
#include "si2c.h"        /* 软件 I2C */
#include "oled.h"        /* OLED 驱动 */
#include "button.h"      /* 按键驱动(自带消抖/单击/连击/长按) */
#include "usart.h"       /* 串口驱动 */
#include "../my_app/cn_font_data.h"   /* 16x16 中文点阵字库（my_app/gen_cn_font.py 生成） */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ==========================================================================
 *                        一、硬件配置区（改接线只改这里）
 * ========================================================================== */

/* ---------- OLED：软件 I2C ---------- */
#define OLED_SCL_GPIO        GPIOB
#define OLED_SCL_PIN         GPIO_Pin_8      /* PB8 -> OLED SCL */
#define OLED_SDA_GPIO        GPIOB
#define OLED_SDA_PIN         GPIO_Pin_9      /* PB9 -> OLED SDA */

/* ---------- ADC 通道 ---------- */
#define LIGHT_ADC_CH         ADC_Channel_0   /* PA0 <- 光敏模块 AO */
#define POT_ADC_CH           ADC_Channel_1   /* PA1 <- 电位器中间脚 */

/* ---------- 蜂鸣器：TIM4_CH1 = PB6 ---------- */
#define BUZZER_TIM           TIM4
#define BUZZER_CH            TIM_Channel_1
#define BUZZER_GPIO          GPIOB
#define BUZZER_PIN           GPIO_Pin_6
#define BUZZER_TONE_HZ       3000            /* 报警音频率 */

/* 蜂鸣器触发方式：1 = 低电平触发（你的模块印着 "Low level trigger"，用 1）
 *                 0 = 高电平触发
 */
#define BUZZER_ACTIVE_LOW    1

/* ---------- 板载 LED ---------- */
#define LED_GPIO             GPIOC
#define LED_PIN              GPIO_Pin_13     /* 蓝板自带，低电平点亮 */
#define LED_ON()             GPIO_ResetBits(LED_GPIO, LED_PIN)
#define LED_OFF()            GPIO_SetBits(LED_GPIO, LED_PIN)

/* ---------- 四个按键（一端接引脚，一端接 GND；内部上拉，按下=低电平） ---------- */
#define KEY1_GPIO            GPIOB
#define KEY1_PIN             GPIO_Pin_12     /* K1 翻页 / 长按静音 */
#define KEY2_GPIO            GPIOB
#define KEY2_PIN             GPIO_Pin_13     /* K2 阈值 + */
#define KEY3_GPIO            GPIOB
#define KEY3_PIN             GPIO_Pin_14     /* K3 阈值 - */
#define KEY4_GPIO            GPIOB
#define KEY4_PIN             GPIO_Pin_15     /* K4 报警开关 / 设置页长按恢复默认阈值 */

/* ---------- 采集与界面参数 ---------- */
#define SAMPLE_PERIOD_MS     20              /* ADC 采样周期 */
#define UI_PERIOD_MS         200             /* 屏幕刷新 + 串口上报周期 */
#define CURVE_PERIOD_MS      250             /* 曲线采样周期 */
#define STAT_PERIOD_MS       10000           /* 每 10 秒往统计里加一个样本 */
#define CURVE_POINTS         128             /* 曲线点数（正好一屏宽，约 32 秒） */
#define STAT_MAX             256             /* RAM 里最多存 256 个样本（约 43 分钟） */
#define ADC_FULL_SCALE       4095

#define TH_STEP_COARSE       100             /* 单击阈值步进 */
#define TH_STEP_FINE         20              /* 长按阈值步进 */
#define TH_DEFAULT           800             /* 默认阈值；设置页里长按 K4 恢复到这个值 */
#define TH_MIN               50
#define TH_MAX               4050

/* ---------- 光敏模块方向（按你的实机测试校准） ----------
 * LIGHT_BIG_IS_BRIGHT = 0：光照越强，ADC 原始值越小   <-- 你的模块是这种
 *                        程序会把它反过来算，所以界面上的"光照"数
 *                        越大 = 环境越亮，越小 = 环境越暗
 *                        如果哪天换成方向相反的模块，改成 1 即可
 * ALARM_WHEN_DIM      = 1：光照低于阈值时报警（天黑报警）   <-- 你要的效果
 *                        想改成"太亮报警"就设为 0
 */
#define LIGHT_BIG_IS_BRIGHT  0
#define ALARM_WHEN_DIM       1

/* ---------- 页面编号 ---------- */
#define PAGE_LIVE            0
#define PAGE_CURVE           1
#define PAGE_SETTING         2
#define PAGE_HISTORY         3
#define PAGE_COUNT           4

/* ==========================================================================
 *                        二、全局变量
 * ========================================================================== */

static SI2C_TypeDef   g_si2c = {
    .SCL_GPIOx    = OLED_SCL_GPIO,
    .SCL_GPIO_Pin = OLED_SCL_PIN,
    .SDA_GPIOx    = OLED_SDA_GPIO,
    .SDA_GPIO_Pin = OLED_SDA_PIN,
};

static OLED_TypeDef   g_oled;

static uint16_t       g_lightRaw  = 0;              /* 光敏原始 ADC 值 */
static uint16_t       g_bright    = 0;              /* 换算后的亮度(越大越亮) */
static uint16_t       g_pot       = 0;              /* 电位器 ADC 值 */
static uint16_t       g_threshold = TH_DEFAULT;     /* 报警阈值(亮度) */
static uint8_t        g_alarmEn   = 1;              /* 报警使能 */
static uint8_t        g_alarmOn   = 0;              /* 当前是否处于报警 */
static uint8_t        g_page      = PAGE_LIVE;
static uint32_t       g_muteUntil = 0;              /* 静音截止时刻 */

static uint16_t       g_curve[CURVE_POINTS];        /* 亮度曲线缓冲 */
static uint16_t       g_curveLen  = 0;

/* ---------- 历史统计：环形缓冲，放在 RAM 里，掉电丢失 ---------- */
static uint16_t       g_statBuf[STAT_MAX];
static uint16_t       g_statCount = 0;              /* 已存样本数(最多 STAT_MAX) */
static uint16_t       g_statWp    = 0;              /* 写指针 */
static uint16_t       g_statMin   = 0;
static uint16_t       g_statMax   = 0;
static uint16_t       g_statAvg   = 0;
static uint8_t        g_statDirty = 1;              /* 1 = 需要重新统计 */
static uint32_t       g_clearTick = 0;              /* "已清除"提示的显示时刻 */

static Button_TypeDef g_keys[4];                    /* K1~K4 */

/* ---------- 前置声明（后面几节会用到） ---------- */
static void     Stat_Clear(void);
static void     Key_Threshold_Reset(void);

/* ==========================================================================
 *                        三、板载 LED
 * ========================================================================== */

static void Led_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    gpio.GPIO_Pin   = LED_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(LED_GPIO, &gpio);

    LED_OFF();
}

/* ==========================================================================
 *                        四、无源蜂鸣器（TIM4_CH1 输出 PWM 方波）
 *   无源蜂鸣器没有自己的振荡电路，必须给它一个方波才会响，
 *   方波的频率决定音调，所以这里用 PWM 来产生方波。
 * ========================================================================== */

static uint16_t g_buzzerHalf = 0;   /* 当前音调对应的半周期比较值 */

static void Buzzer_SetFreq(uint16_t freq)
{
    uint16_t arr;

    if (freq < 100)   freq = 100;
    if (freq > 20000) freq = 20000;

    /* 定时器计数时钟先分频到 1MHz，所以 ARR = 1000000/频率 - 1 */
    arr = (uint16_t)(1000000UL / freq - 1);
    g_buzzerHalf = (uint16_t)((arr + 1) / 2);

    TIM_SetAutoreload(BUZZER_TIM, arr);
    TIM_SetCompare1(BUZZER_TIM, g_buzzerHalf);                /* 50% 方波 = 发声 */
}

/* 发声：输出 50% 的方波 */
static void Buzzer_On(void)
{
    TIM_SetCompare1(BUZZER_TIM, g_buzzerHalf);
}

/* 静音：比较值给 0。
 * 你的模块是"低电平触发"，所以需要一根一直保持高电平的信号才算安静，
 * 这就是下面初始化里用 PWM 模式2 的原因（高电平触发的模块则用模式1）。
 * 注意通道不能关，否则引脚悬空，模块可能乱响。 */
static void Buzzer_Off(void)
{
    TIM_SetCompare1(BUZZER_TIM, 0);
}

static void Buzzer_Init(void)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef       oc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    /* PB6 = TIM4_CH1，复用推挽输出 */
    gpio.GPIO_Pin   = BUZZER_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BUZZER_GPIO, &gpio);

    /* 72MHz / 72 = 1MHz 计数时钟 */
    tim.TIM_Prescaler         = 72 - 1;
    tim.TIM_CounterMode       = TIM_CounterMode_Up;
    tim.TIM_Period            = (uint16_t)(1000000UL / BUZZER_TONE_HZ - 1);
    tim.TIM_ClockDivision     = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(BUZZER_TIM, &tim);

#if (BUZZER_ACTIVE_LOW)
    oc.TIM_OCMode       = TIM_OCMode_PWM2;    /* 低电平触发：比较值0时输出恒高=静音 */
#else
    oc.TIM_OCMode       = TIM_OCMode_PWM1;    /* 高电平触发：比较值0时输出恒低=静音 */
#endif
    oc.TIM_OutputState  = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputNState_Disable;
    oc.TIM_Pulse        = 0;                  /* 上电先静音 */
    oc.TIM_OCPolarity   = TIM_OCPolarity_High;
    oc.TIM_OCNPolarity  = TIM_OCNPolarity_High;
    oc.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC1Init(BUZZER_TIM, &oc);
    TIM_OC1PreloadConfig(BUZZER_TIM, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(BUZZER_TIM, ENABLE);

    TIM_Cmd(BUZZER_TIM, ENABLE);
    Buzzer_SetFreq(BUZZER_TONE_HZ);
    Buzzer_Off();                       /* 先保持安静 */
}

/* 短促的提示音（会阻塞 ms 毫秒，只用于按键、开机提示） */
static void Beep(uint16_t freq, uint16_t ms)
{
    Buzzer_SetFreq(freq);
    Buzzer_On();
    Delay(ms);
    Buzzer_Off();
}

/* ==========================================================================
 *                        五、ADC（光敏 + 电位器）
 * ========================================================================== */

static void Adc_Init(void)
{
    GPIO_InitTypeDef gpio;
    ADC_InitTypeDef  adc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);

    /* ADC 时钟 = PCLK2/6 = 72/6 = 12MHz（最大不能超过 14MHz） */
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    /* PA0 / PA1 设为模拟输入 */
    gpio.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &gpio);

    ADC_DeInit(ADC1);

    adc.ADC_Mode               = ADC_Mode_Independent;
    adc.ADC_ScanConvMode       = DISABLE;              /* 单通道、软件触发 */
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign          = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel       = 1;
    ADC_Init(ADC1, &adc);

    ADC_Cmd(ADC1, ENABLE);

    /* ADC 自校准（官方推荐流程，能明显提高精度） */
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1) == SET);
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1) == SET);
}

/* 读一次指定通道的原始值（0~4095） */
static uint16_t Adc_ReadOnce(uint8_t channel)
{
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_55Cycles5);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    return ADC_GetConversionValue(ADC1);
}

/* 连续读 times 次取平均，滤掉高频噪声 */
static uint16_t Adc_ReadAverage(uint8_t channel, uint8_t times)
{
    uint32_t sum = 0;

    for (uint8_t i = 0; i < times; i++)
    {
        sum += Adc_ReadOnce(channel);
    }
    return (uint16_t)(sum / times);
}

/* 采样 + 一阶低通滤波，每 20ms 调用一次 */
static void Sample_Update(void)
{
    uint16_t rawLight = Adc_ReadAverage(LIGHT_ADC_CH, 8);
    uint16_t rawPot   = Adc_ReadAverage(POT_ADC_CH, 8);

    g_lightRaw = (uint16_t)(g_lightRaw + (((int32_t)rawLight - (int32_t)g_lightRaw) / 4));
    g_pot      = (uint16_t)(g_pot      + (((int32_t)rawPot   - (int32_t)g_pot)      / 4));

#if (LIGHT_BIG_IS_BRIGHT)
    g_bright = g_lightRaw;
#else
    g_bright = (uint16_t)(ADC_FULL_SCALE - g_lightRaw);   /* 反向模块：翻过来算 */
#endif
}

/* 当前亮度是否满足报警条件 */
static uint8_t Bright_IsAlarming(void)
{
#if (ALARM_WHEN_DIM)
    return (g_bright < g_threshold) ? 1 : 0;    /* 天黑报警 */
#else
    return (g_bright > g_threshold) ? 1 : 0;    /* 太亮报警 */
#endif
}

/* ==========================================================================
 *                        六、按键（调用 my_lib 的 button 驱动）
 * ========================================================================== */

static void Key_Threshold_Add(int16_t step)
{
    int32_t v = (int32_t)g_threshold + step;

    if (v > TH_MAX) v = TH_MAX;
    if (v < TH_MIN) v = TH_MIN;
    g_threshold = (uint16_t)v;
}

/* 恢复默认阈值（不用 Flash，所以阈值只存在内存里，掉电会回到默认值） */
static void Key_Threshold_Reset(void)
{
    g_threshold = TH_DEFAULT;
}

static void Key1_Clicked(uint8_t clicks)
{
    if (clicks == 1)
    {
        g_page = (uint8_t)((g_page + 1) % PAGE_COUNT);              /* 单击：下一页 */
    }
    else
    {
        g_page = (uint8_t)((g_page + PAGE_COUNT - 1) % PAGE_COUNT); /* 连击：上一页 */
    }
    Beep(4000, 25);
}

static void Key1_LongPressed(uint8_t ticks)
{
    if (ticks == 1)                     /* 长按：静音 30 秒 */
    {
        g_muteUntil = GetTick() + 30000;
        Beep(1500, 60);
    }
}

static void Key2_Clicked(uint8_t clicks)
{
    if (g_page == PAGE_SETTING)
    {
        Key_Threshold_Add((int16_t)(TH_STEP_COARSE * clicks));
        Beep(4000, 25);
    }
}

static void Key2_LongPressed(uint8_t ticks)
{
    if (g_page == PAGE_SETTING)         /* 长按连续加，越按越快 */
    {
        Key_Threshold_Add((int16_t)(TH_STEP_FINE * ((ticks > 3) ? 3 : 1)));
    }
}

static void Key3_Clicked(uint8_t clicks)
{
    if (g_page == PAGE_SETTING)
    {
        Key_Threshold_Add((int16_t)(-(TH_STEP_COARSE * clicks)));
        Beep(4000, 25);
    }
}

static void Key3_LongPressed(uint8_t ticks)
{
    if (ticks != 1) return;

    if (g_page == PAGE_SETTING)
    {
        Key_Threshold_Add(-TH_STEP_FINE);
    }
    else if (g_page == PAGE_HISTORY)
    {
        Stat_Clear();                   /* 长按 K3：清空本次上电的统计 */
        Beep(1000, 150);
    }
}

static void Key4_Clicked(uint8_t clicks)
{
    g_alarmEn = g_alarmEn ? 0 : 1;      /* 单击：报警开关 */
    Beep(g_alarmEn ? 3000 : 1200, 40);
}

static void Key4_LongPressed(uint8_t ticks)
{
    if (ticks != 1) return;

    if (g_page == PAGE_SETTING)         /* 设置页长按：阈值恢复默认 */
    {
        Key_Threshold_Reset();
        Beep(2500, 60);
    }
}

static void Keys_Init(void)
{
    Button_InitTypeDef cfg;

    memset(&cfg, 0, sizeof(cfg));

    /* 公共参数：长按 800ms 触发，之后每 150ms 再触发，连击间隔 250ms */
    cfg.LongPressTime         = 800;
    cfg.LongPressTickInterval = 150;
    cfg.ClickInterval         = 250;

    /* ---- K1 ---- */
    cfg.GPIOx = KEY1_GPIO;
    cfg.GPIO_Pin = KEY1_PIN;
    cfg.button_clicked_cb = Key1_Clicked;
    cfg.button_long_pressed_cb = Key1_LongPressed;
    My_Button_Init(&g_keys[0], &cfg);

    /* ---- K2 ---- */
    cfg.GPIOx = KEY2_GPIO;
    cfg.GPIO_Pin = KEY2_PIN;
    cfg.button_clicked_cb = Key2_Clicked;
    cfg.button_long_pressed_cb = Key2_LongPressed;
    My_Button_Init(&g_keys[1], &cfg);

    /* ---- K3 ---- */
    cfg.GPIOx = KEY3_GPIO;
    cfg.GPIO_Pin = KEY3_PIN;
    cfg.button_clicked_cb = Key3_Clicked;
    cfg.button_long_pressed_cb = Key3_LongPressed;
    My_Button_Init(&g_keys[2], &cfg);

    /* ---- K4 ---- */
    cfg.GPIOx = KEY4_GPIO;
    cfg.GPIO_Pin = KEY4_PIN;
    cfg.button_clicked_cb = Key4_Clicked;
    cfg.button_long_pressed_cb = Key4_LongPressed;
    My_Button_Init(&g_keys[3], &cfg);
}

/* ==========================================================================
 *                        七、串口 USART1（115200 8N1）
 * ========================================================================== */

static void Usart1_Init(void)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

    /* PA9 -> TX，复用推挽输出 */
    gpio.GPIO_Pin   = GPIO_Pin_9;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* PA10 <- RX，浮空输入 */
    gpio.GPIO_Pin  = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    usart.USART_BaudRate            = 115200;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &usart);

    USART_Cmd(USART1, ENABLE);
}

/* 每 200ms 上报一行：光照,电位器,阈值,报警开关,统计条数
 * 串口助手勾选"时间戳"就能当数据记录用，也可以直接存成 CSV 画曲线 */
static void Serial_Report(void)
{
    My_USART_Printf(USART1, "%u,%u,%u,%u,%u\r\n",
                    g_bright, g_pot, g_threshold, g_alarmEn, g_statCount);
}

/* 串口命令：p=立即上报一行  r=清空统计  s=阈值恢复默认 */
static void Serial_Poll(void)
{
    char c;

    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET) return;

    c = (char)USART_ReceiveData(USART1);

    switch (c)
    {
        case 'p':
        case 'P':
            Serial_Report();
            break;

        case 'r':
        case 'R':
            Stat_Clear();
            break;

        case 's':
        case 'S':
            Key_Threshold_Reset();
            break;

        default:
            break;
    }
}

/* ==========================================================================
 *                        八、历史统计（RAM 环形缓冲，掉电丢失）
 * ========================================================================== */

/* 加一个样本：存满 STAT_MAX 个以后从头覆盖最老的 */
static void Stat_Add(uint16_t value)
{
    g_statBuf[g_statWp] = value;
    g_statWp = (uint16_t)((g_statWp + 1) % STAT_MAX);

    if (g_statCount < STAT_MAX)
    {
        g_statCount++;
    }
    g_statDirty = 1;
}

/* 清空统计（K3 长按 / 串口 r） */
static void Stat_Clear(void)
{
    g_statCount = 0;
    g_statWp    = 0;
    g_statDirty = 1;
    g_clearTick = GetTick();
}

/* 统计最大值 / 最小值 / 平均值（只在需要时算一次） */
static void Stat_Calc(void)
{
    uint16_t i;
    uint16_t idx;
    uint16_t minV = 0xFFFF;
    uint16_t maxV = 0;
    uint32_t sum  = 0;

    if (g_statCount == 0)
    {
        g_statMin   = 0;
        g_statMax   = 0;
        g_statAvg   = 0;
        g_statDirty = 0;
        return;
    }

    /* 最老的一个样本的位置 */
    idx = (uint16_t)((g_statWp + STAT_MAX - g_statCount) % STAT_MAX);

    for (i = 0; i < g_statCount; i++)
    {
        uint16_t v = g_statBuf[idx];

        if (v < minV) minV = v;
        if (v > maxV) maxV = v;
        sum += v;

        idx = (uint16_t)((idx + 1) % STAT_MAX);
    }

    g_statMin   = minV;
    g_statMax   = maxV;
    g_statAvg   = (uint16_t)(sum / g_statCount);
    g_statDirty = 0;
}

/* ==========================================================================
 *                        九、OLED 界面
 * ========================================================================== */

static int Oled_I2cWrite(uint8_t addr, const uint8_t *pdata, uint16_t size)
{
    return My_SI2C_SendBytes(&g_si2c, addr, pdata, size);
}

/* 在 (x, yBase) 处画字符串，yBase 是字符底部的 y 坐标（8px 英文/数字） */
static void Ui_Text(int16_t x, int16_t yBase, const char *str)
{
    OLED_SetPen(&g_oled, PEN_COLOR_WHITE, 1);
    OLED_SetBrush(&g_oled, BRUSH_TRANSPARENT);
    OLED_SetCursor(&g_oled, x, yBase);
    OLED_DrawString(&g_oled, str);
}

static void Ui_Printf(int16_t x, int16_t yBase, const char *fmt, ...)
{
    char    buf[40];
    va_list ap;

    va_start(ap, fmt);
    vsprintf(buf, fmt, ap);
    va_end(ap);

    Ui_Text(x, yBase, buf);
}

/* --------------------------------------------------------------------------
 *  中文显示（16x16 点阵，字库在 my_app/cn_font_data.h 里）
 *  左上角定位：yTop 就是字形最上面一行的 y 坐标
 * ------------------------------------------------------------------------ */

static const CnGlyph_TypeDef *Cn_FindGlyph(uint32_t code)
{
    uint16_t i;

    for (i = 0; i < CN_GLYPH_COUNT; i++)
    {
        if (g_cnGlyphs[i].code == code) return &g_cnGlyphs[i];
    }
    return 0;
}

/* 取下一个 UTF-8 字符的 Unicode 编码 */
static uint32_t Utf8_Next(const uint8_t **ppStr)
{
    const uint8_t *s = *ppStr;
    uint32_t       code;

    if (s[0] < 0x80)
    {
        code = s[0];
        *ppStr = s + 1;
    }
    else if ((s[0] & 0xE0) == 0xC0)
    {
        code = ((uint32_t)(s[0] & 0x1F) << 6) | (uint32_t)(s[1] & 0x3F);
        *ppStr = s + 2;
    }
    else if ((s[0] & 0xF0) == 0xE0)
    {
        code = ((uint32_t)(s[0] & 0x0F) << 12)
             | ((uint32_t)(s[1] & 0x3F) << 6)
             | (uint32_t)(s[2] & 0x3F);
        *ppStr = s + 3;
    }
    else
    {
        code = '?';
        *ppStr = s + 1;
    }
    return code;
}

/* 画一个汉字，返回下一个字的 x 坐标；invert=1 时画成白底黑字 */
static int16_t Ui_CnChar(int16_t x, int16_t yTop, uint32_t code, uint8_t invert)
{
    const CnGlyph_TypeDef *pGlyph = Cn_FindGlyph(code);

    if (invert)
    {
        OLED_SetPen(&g_oled, PEN_COLOR_BLACK, 1);
        OLED_SetBrush(&g_oled, BRUSH_WHITE);
    }
    else
    {
        OLED_SetPen(&g_oled, PEN_COLOR_WHITE, 1);
        OLED_SetBrush(&g_oled, BRUSH_TRANSPARENT);
    }

    if (pGlyph != 0)
    {
        OLED_SetCursor(&g_oled, x, yTop);
        OLED_DrawBitmap(&g_oled, 16, 16, pGlyph->bmp);
    }

    OLED_SetPen(&g_oled, PEN_COLOR_WHITE, 1);
    OLED_SetBrush(&g_oled, BRUSH_TRANSPARENT);

    return (int16_t)(x + 16);
}

/* 画一串中文，返回结束时的 x 坐标 */
static int16_t Ui_CnText(int16_t x, int16_t yTop, const char *str)
{
    const uint8_t *s = (const uint8_t *)str;

    while (*s != 0)
    {
        x = Ui_CnChar(x, yTop, Utf8_Next(&s), 0);
    }
    return x;
}

/* 反白横幅：整行刷白 + 黑字，用于报警提示 */
static void Ui_CnBanner(int16_t yTop, const char *str)
{
    const uint8_t *s = (const uint8_t *)str;
    int16_t        x = 34;

    OLED_SetPen(&g_oled, PEN_COLOR_TRANSPARENT, 1);
    OLED_SetBrush(&g_oled, BRUSH_WHITE);
    OLED_SetCursor(&g_oled, 0, yTop);
    OLED_DrawRect(&g_oled, 128, 16);

    while (*s != 0)
    {
        x = Ui_CnChar(x, yTop, Utf8_Next(&s), 1);
    }

    OLED_SetPen(&g_oled, PEN_COLOR_WHITE, 1);
    OLED_SetBrush(&g_oled, BRUSH_TRANSPARENT);
}

/* 画一根进度条：外框 + 内部填充 */
static void Ui_Bar(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t value, uint16_t maxValue)
{
    uint16_t fill;

    OLED_SetPen(&g_oled, PEN_COLOR_WHITE, 1);
    OLED_SetBrush(&g_oled, BRUSH_TRANSPARENT);
    OLED_SetCursor(&g_oled, x, y);
    OLED_DrawRect(&g_oled, w, h);

    fill = (uint16_t)((uint32_t)value * (uint32_t)(w - 2) / maxValue);
    if (fill > (uint16_t)(w - 2)) fill = (uint16_t)(w - 2);

    OLED_SetPen(&g_oled, PEN_COLOR_TRANSPARENT, 1);
    OLED_SetBrush(&g_oled, BRUSH_WHITE);
    OLED_SetCursor(&g_oled, x + 1, y + 1);
    OLED_DrawRect(&g_oled, fill, (uint16_t)(h - 2));

    OLED_SetPen(&g_oled, PEN_COLOR_WHITE, 1);
    OLED_SetBrush(&g_oled, BRUSH_TRANSPARENT);
}

/* 第 1 页：实时值 */
static void Ui_PageLive(void)
{
    /* 第 1 行：标题；报警时这一行变成反白横幅 */
    if (g_alarmOn)
    {
        Ui_CnBanner(0, "报警中");
    }
    else
    {
        Ui_CnText(0, 0, "光照监测");
        Ui_Text(100, 15, "1/4");

        if ((int32_t)(g_muteUntil - GetTick()) > 0)
        {
            Ui_CnText(64, 0, "静音");
        }
    }

    /* 第 2 行：光照亮度 + 柱状条 */
    Ui_CnText(0, 16, "光照");
    Ui_Bar(36, 20, 62, 8, g_bright, ADC_FULL_SCALE);
    Ui_Printf(100, 31, "%4u", g_bright);

    /* 第 3 行：报警阈值 + 柱状条（和上面的光照条对比，一眼看出离报警还差多少） */
    Ui_CnText(0, 32, "阈值");
    Ui_Bar(36, 36, 62, 8, g_threshold, ADC_FULL_SCALE);
    Ui_Printf(100, 47, "%4u", g_threshold);

    /* 第 4 行：电位器读数 + 报警开关 */
    Ui_CnText(0, 48, "电位");
    Ui_Printf(36, 63, "%4u", g_pot);
    Ui_CnText(68, 48, "报警");
    Ui_CnText(104, 48, g_alarmEn ? "开" : "关");
}

/* 第 2 页：亮度曲线（128 点，约 32 秒） */
static void Ui_PageCurve(void)
{
    const int16_t top    = 20;
    const int16_t bottom = 53;
    int16_t prevY = bottom;
    uint16_t i;

    Ui_CnText(0, 0, "亮度曲线");
    Ui_Text(100, 15, "2/4");

    /* 曲线区域的边框：y 17 ~ 55 */
    OLED_SetPen(&g_oled, PEN_COLOR_WHITE, 1);
    OLED_SetBrush(&g_oled, BRUSH_TRANSPARENT);
    OLED_SetCursor(&g_oled, 0, 17);
    OLED_DrawRect(&g_oled, 128, 39);

    for (i = 0; i < CURVE_POINTS; i++)
    {
        uint16_t v = (i < g_curveLen) ? g_curve[i] : 0;
        int16_t  y = (int16_t)(bottom - (int32_t)v * (bottom - top) / ADC_FULL_SCALE);

        OLED_SetCursor(&g_oled, (int16_t)i, y);
        OLED_DrawDot(&g_oled);                       /* 画点 */

        OLED_SetCursor(&g_oled, (int16_t)i, prevY);
        OLED_DrawLine(&g_oled, (int16_t)i, y);       /* 和上一个点连起来 */

        prevY = y;
    }

    Ui_Printf(0, 64, "now:%4u  len:%3u", g_bright, g_curveLen);
}

/* 第 3 页：参数设置 */
static void Ui_PageSetting(void)
{
    Ui_CnText(0, 0, "参数设置");
    Ui_Text(100, 15, "3/4");

    Ui_CnText(0, 16, "阈值");
    Ui_Bar(36, 20, 62, 8, g_threshold, ADC_FULL_SCALE);
    Ui_Printf(100, 31, "%4u", g_threshold);

    Ui_CnText(0, 32, "报警");
    Ui_CnText(36, 32, g_alarmEn ? "开" : "关");

    /* 操作提示：K2/K3 加减阈值，长按 K4 恢复默认阈值 */
    Ui_Text(0, 64, "K2K3 +/-  K4 DEFLT");
}

/* 第 4 页：历史统计（本次上电后，存在 RAM 里） */
static void Ui_PageHistory(void)
{
    if (g_statDirty)
    {
        Stat_Calc();
    }

    Ui_CnText(0, 0, "历史记录");
    Ui_Text(100, 15, "4/4");

    /* 第 2 行：共 N 条 + 平均 */
    Ui_CnText(0, 16, "共");
    Ui_Printf(20, 31, "%u", g_statCount);
    Ui_CnText(48, 16, "条");
    Ui_CnText(64, 16, "平均");
    Ui_Printf(100, 31, "%4u", g_statAvg);

    /* 第 3 行：最大 / 最小 */
    Ui_CnText(0, 32, "最大");
    Ui_Printf(34, 47, "%4u", g_statMax);
    Ui_CnText(66, 32, "最小");
    Ui_Printf(100, 47, "%4u", g_statMin);

    /* 第 4 行：清空结果提示（显示 2 秒），平时显示操作提示 */
    if (g_clearTick != 0 && (uint32_t)(GetTick() - g_clearTick) < 2000)
    {
        Ui_CnText(0, 48, "已清除");
    }
    else
    {
        Ui_Text(0, 64, "Hold K3 clear");
    }
}

/* 每 200ms 重画整屏（先清缓冲区，避免残留旧内容） */
static void Ui_Draw(void)
{
    OLED_Clear(&g_oled);

    switch (g_page)
    {
        case PAGE_CURVE:
            Ui_PageCurve();
            break;

        case PAGE_SETTING:
            Ui_PageSetting();
            break;

        case PAGE_HISTORY:
            Ui_PageHistory();
            break;

        default:
            Ui_PageLive();
            break;
    }
}

/* ==========================================================================
 *                        十、报警与指示灯
 * ========================================================================== */

static void Alarm_Service(void)
{
    uint32_t now = GetTick();
    uint32_t period;
    uint32_t onTime;
    uint16_t diff;

    /* 1. 判断当前是否需要报警 */
    g_alarmOn = (g_alarmEn && Bright_IsAlarming()) ? 1 : 0;

    /* 2. 不报警 / 静音期间：蜂鸣器保持安静 */
    if (!g_alarmOn || ((int32_t)(g_muteUntil - now) > 0))
    {
        Buzzer_Off();
        return;
    }

    /* 3. 离阈值越远，叫得越急 */
    period = 600;
    onTime = 120;
    diff   = (g_threshold > g_bright) ? (uint16_t)(g_threshold - g_bright)
                                      : (uint16_t)(g_bright - g_threshold);
    if (diff > (uint16_t)(g_threshold / 2))
    {
        period = 250;
        onTime = 90;
    }

    if ((now % period) < onTime)
    {
        Buzzer_SetFreq(BUZZER_TONE_HZ);
        Buzzer_On();
    }
    else
    {
        Buzzer_Off();
    }
}

static void Led_Service(void)
{
    uint32_t now = GetTick();

    if (g_alarmOn)
    {
        if ((now % 200) < 100) LED_ON();
        else                   LED_OFF();
    }
    else
    {
        if ((now % 2000) < 30) LED_ON();           /* 正常时 2 秒闪一下 */
        else                   LED_OFF();
    }
}

/* ==========================================================================
 *                        十一、main
 * ========================================================================== */

int main(void)
{
    OLED_InitTypeDef oledCfg;
    uint32_t tSample, tCurve, tStat, tUi;
    uint8_t  oledOk = 0;
    uint8_t  i;

    /* ---- 1. 基础外设初始化 ---- */
    Delay_Init();
    Led_Init();
    Buzzer_Init();
    Adc_Init();
    Keys_Init();
    Usart1_Init();

    /* ---- 2. OLED 初始化（接线不好会失败，重试 3 次） ---- */
    My_SI2C_Init(&g_si2c);
    oledCfg.i2c_write_cb = Oled_I2cWrite;

    for (i = 0; i < 3; i++)
    {
        if (OLED_Init(&g_oled, &oledCfg) == 0)
        {
            oledOk = 1;
            break;
        }
        Delay(50);
    }

    if (!oledOk)
    {
        /* 屏幕没接好：蜂鸣器长鸣 + LED 快闪，方便排查 */
        Beep(2000, 500);
        while (1)
        {
            LED_ON();
            Delay(80);
            LED_OFF();
            Delay(200);
        }
    }

    OLED_Clear(&g_oled);
    OLED_SendBuffer(&g_oled);

    /* ---- 3. 开机画面 + 串口欢迎信息 ---- */
    Ui_CnText(0, 0, "光照监测");
    Ui_Text(100, 15, "v1.1");
    Ui_Text(0, 31, "STM32F103C8T6");
    Ui_Text(0, 47, "ADC PA0 PA1");
    Ui_Text(0, 63, "PWM PB6  LCD PB8/9");
    OLED_SendBuffer(&g_oled);

    My_USART_Printf(USART1, "\r\n=== ENV MONITOR v1.1 (no Flash) ===\r\n");
    My_USART_Printf(USART1, "CSV: bright,pot,threshold,alarm_en,stat_count\r\n");

    Beep(4000, 80);
    Delay(100);
    Beep(5000, 120);
    Delay(700);

    Sample_Update();

    /* ---- 4. 主循环：按时间片做不同的事情 ---- */
    tSample = GetTick();
    tCurve  = tSample;
    tStat   = tSample;
    tUi     = 0;

    while (1)
    {
        uint32_t now = GetTick();

        /* 20ms：采集 + 滤波 */
        if ((uint32_t)(now - tSample) >= SAMPLE_PERIOD_MS)
        {
            tSample = now;
            Sample_Update();
        }

        /* 250ms：记录一个曲线点 */
        if ((uint32_t)(now - tCurve) >= CURVE_PERIOD_MS)
        {
            tCurve = now;
            if (g_curveLen < CURVE_POINTS)
            {
                g_curve[g_curveLen++] = g_bright;
            }
            else
            {
                memmove(&g_curve[0], &g_curve[1], (CURVE_POINTS - 1) * sizeof(uint16_t));
                g_curve[CURVE_POINTS - 1] = g_bright;
            }
        }

        /* 10s：往统计里加一个样本 */
        if ((uint32_t)(now - tStat) >= STAT_PERIOD_MS)
        {
            tStat = now;
            Stat_Add(g_bright);
        }

        /* 按键轮询（驱动要求频繁调用） */
        for (i = 0; i < 4; i++)
        {
            My_Button_Proc(&g_keys[i]);
        }

        /* 报警判断 + 蜂鸣器节奏 + LED 指示 */
        Alarm_Service();
        Led_Service();

        /* 200ms：刷新屏幕 + 串口上报 */
        if ((uint32_t)(now - tUi) >= UI_PERIOD_MS)
        {
            tUi = now;
            Ui_Draw();
            OLED_SendBuffer(&g_oled);
            Serial_Report();
            Serial_Poll();
        }
    }
}

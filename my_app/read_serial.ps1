# ============================================================================
#  读取项目一的串口数据（不用装串口助手软件，Windows 自带 PowerShell 就能跑）
#
#  用法（在项目文件夹 C:\Users\luo20\Desktop\stm32模板 下打开 PowerShell）：
#      powershell -ExecutionPolicy Bypass -File my_app\read_serial.ps1
#      powershell -ExecutionPolicy Bypass -File my_app\read_serial.ps1 -Port COM3
#
#  它会：
#    1) 列出电脑上的串口（没有参数时让你输入 COM 号）
#    2) 以 115200 8N1 打开串口，屏幕上实时显示每一行
#    3) 同时把数据追加保存到 my_app\data.csv（可以用 Excel 打开画图）
#
#  退出：按 Ctrl+C
# ============================================================================

param(
    [string]$Port = "",
    [int]$Baud = 115200
)

$ErrorActionPreference = "Stop"
$csvPath = Join-Path $PSScriptRoot "data.csv"

if (-not $Port) {
    $ports = [System.IO.Ports.SerialPort]::GetPortNames()
    if ($ports.Count -eq 0) {
        Write-Host ""
        Write-Host "没有检测到任何串口。" -ForegroundColor Yellow
        Write-Host "请检查："
        Write-Host "  1) USB 转 TTL 模块插到电脑上了吗（模块上的小灯应该亮）"
        Write-Host "  2) 装过 CH340 驱动吗（设备管理器 -> 端口 里应该能看到 USB-SERIAL CH340）"
        Write-Host ""
        exit 1
    }
    Write-Host ("可用串口: " + ($ports -join ", ")) -ForegroundColor Cyan
    $Port = Read-Host "请输入串口号（例如 COM3）"
}

$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, "None", 8, "One"
$sp.ReadTimeout  = 3000
$sp.NewLine      = "`n"

try {
    $sp.Open()
}
catch {
    Write-Host ("打不开串口 " + $Port + "：" + $_.Exception.Message) -ForegroundColor Red
    Write-Host "常见原因：COM 号填错、被别的软件占用了（先关掉串口助手/XCOM 等）"
    exit 1
}

Write-Host ("已打开 " + $Port + " @ " + $Baud + " 8N1，按 Ctrl+C 退出") -ForegroundColor Green
Write-Host ("数据同时保存到: " + $csvPath) -ForegroundColor DarkGray
Write-Host ""
Write-Host "光照,电位器,阈值,报警开关,统计条数"

if (-not (Test-Path $csvPath)) {
    "bright,pot,threshold,alarm_en,stat_count" | Out-File -FilePath $csvPath -Encoding ascii
}

while ($true) {
    try {
        $line = $sp.ReadLine().Trim()
        if ($line.Length -gt 0) {
            Write-Host $line
            Add-Content -Path $csvPath -Value $line -Encoding ascii
        }
    }
    catch [System.TimeoutException] {
        # 3 秒没收到数据不报错，继续等
    }
}

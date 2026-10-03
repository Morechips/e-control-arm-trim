param([switch]$DisableArmTrim)

$ErrorActionPreference = 'Stop'

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$outputDirectory = Join-Path $projectRoot 'firmware_direct'
$objectDirectory = Join-Path $outputDirectory 'obj'

$compiler = (Get-Command arm-none-eabi-gcc -ErrorAction Stop).Source
$objectCopy = (Get-Command arm-none-eabi-objcopy -ErrorAction Stop).Source
$sizeTool = (Get-Command arm-none-eabi-size -ErrorAction Stop).Source

$mcuFlags = @(
    '-mcpu=cortex-m4', '-mthumb', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard'
)
$includeFlags = @(
    '-ICore/Inc',
    '-IDrivers/STM32F4xx_HAL_Driver/Inc',
    '-IDrivers/CMSIS/Device/ST/STM32F4xx/Include',
    '-IDrivers/CMSIS/CMSIS/Core/Include'
)
$commonCFlags = @(
    '-std=c11', '-DUSE_HAL_DRIVER', '-DSTM32F407xx',
    ('-DARM_TRIM_ENABLE=' + [int](-not $DisableArmTrim)),
    '-Os', '-g3', '-ffunction-sections', '-fdata-sections',
    '-Wall', '-Wextra', '-Werror'
)
$sources = @(
    'Core/Src/main.c',
    'Core/Src/servo.c',
    'Core/Src/arm_kinematics.c',
    'Core/Src/arm_collision.c',
    'Core/Src/arm_trim.c',
    'Core/Src/arm_trim_project.c',
    'Core/Src/arm_trim_service.c',
    'Core/Src/arm_trim_input.c',
    'Core/Src/board_inputs.c',
    'Core/Src/board_app.c',
    'Core/Src/start_button.c',
    'Core/Src/vision_data.c',
    'Core/Src/maxicam.c',
    'Core/Src/laser.c',
    'Core/Src/turn_right.c',
    'Core/Src/action_fsm.c',
    'Core/Src/mission_fsm.c',
    'Core/Src/route_fsm.c',
    'Core/Src/uart_driver.c',
    'Core/Src/uart_tx_queue.c',
    'Core/Src/serial_io.c',
    'Core/Src/bluetooth_driver.c',
    'Core/Src/motor_driver.c',
    'Core/Src/emm42_driver.c',
    'Core/Src/mecanum.c',
    'Core/Src/mecanum_test.c',
    'Core/Src/car_control.c',
    'Core/Src/remote_heading.c',
    'Core/Src/uart_bridge.c',
    'Core/Src/usart2_dma.c',
    'Core/Src/jy61.c',
    'Core/Src/heading_control.c',
    'Core/Src/pid_tuner.c',
    'Core/Src/ssd1306.c',
    'Core/Src/stm32f4xx_hal_msp.c',
    'Core/Src/stm32f4xx_it.c',
    'cmake/newlib_init.c',
    'Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c',
    'Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c',
    'Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c',
    'Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c',
    'Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c',
    'Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c',
    'Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c',
    'Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c',
    'Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c',
    'Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c',
    'Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c'
)
$assemblySource = 'Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f407xx.s'

New-Item -ItemType Directory -Force -Path $objectDirectory | Out-Null
Push-Location $projectRoot
try {
    $objects = @()
    for ($index = 0; $index -lt $sources.Count; ++$index) {
        $source = $sources[$index]
        $stem = [IO.Path]::GetFileNameWithoutExtension($source)
        $object = Join-Path $objectDirectory ('{0:D2}_{1}.o' -f $index, $stem)
        & $compiler @mcuFlags @commonCFlags @includeFlags -c $source -o $object
        if ($LASTEXITCODE -ne 0) { throw "Compile failed: $source" }
        $objects += $object
    }

    $startupObject = Join-Path $objectDirectory 'startup_stm32f407xx.o'
    & $compiler @mcuFlags -x assembler-with-cpp -DSTM32F407xx @includeFlags `
        -c $assemblySource -o $startupObject
    if ($LASTEXITCODE -ne 0) { throw "Compile failed: $assemblySource" }
    $objects += $startupObject

    $elf = Join-Path $outputDirectory 'stm32f407_bt_oled.elf'
    $hex = Join-Path $outputDirectory 'stm32f407_bt_oled.hex'
    $bin = Join-Path $outputDirectory 'stm32f407_bt_oled.bin'
    $map = Join-Path $outputDirectory 'stm32f407_bt_oled.map'
    & $compiler @mcuFlags @objects '-TSTM32F407_FLASH.ld' '-nostartfiles' `
        '--specs=nano.specs' '--specs=nosys.specs' '-Wl,--gc-sections' `
        "-Wl,-Map=$map" '-lm' -o $elf
    if ($LASTEXITCODE -ne 0) { throw 'Firmware link failed' }

    & $objectCopy -O ihex $elf $hex
    if ($LASTEXITCODE -ne 0) { throw 'HEX generation failed' }
    & $objectCopy -O binary $elf $bin
    if ($LASTEXITCODE -ne 0) { throw 'BIN generation failed' }
    & $sizeTool $elf
    if ($LASTEXITCODE -ne 0) { throw 'Size report failed' }
    # Record the actual input bytes, independent of uncommitted branch state.
    $trimBuildInputs = @($sources + $assemblySource + 'STM32F407_FLASH.ld' + 'scripts/build_firmware.ps1')
    $trimBuildInputs += @(Get-ChildItem -LiteralPath (Join-Path $projectRoot 'Core\Inc') -Filter '*.h' -File |
        ForEach-Object { 'Core/Inc/' + $_.Name })
    $trimBuildInputs += @(Get-ChildItem -LiteralPath (Join-Path $projectRoot 'Drivers') -Filter '*.h' -File -Recurse |
        ForEach-Object { $_.FullName.Substring($projectRoot.Length + 1).Replace('\', '/') })
    $trimBuildInputHashes = [ordered]@{}
    foreach ($trimBuildInput in ($trimBuildInputs | Sort-Object -Unique)) {
        $trimBuildInputHashes[$trimBuildInput] = (Get-FileHash -LiteralPath (Join-Path $projectRoot $trimBuildInput) -Algorithm SHA256).Hash
    }
    $trimBuildManifest = [ordered]@{
        version = 'v4.6-servo-integration'
        built_utc = [DateTime]::UtcNow.ToString('o')
        arm_trim_enabled = (-not $DisableArmTrim)
        compiler = (& $compiler --version | Select-Object -First 1)
        c_flags = @($mcuFlags + $commonCFlags + $includeFlags)
        firmware_sha256 = [ordered]@{
            elf = (Get-FileHash -LiteralPath $elf -Algorithm SHA256).Hash
            hex = (Get-FileHash -LiteralPath $hex -Algorithm SHA256).Hash
            bin = (Get-FileHash -LiteralPath $bin -Algorithm SHA256).Hash
        }
        source_sha256 = $trimBuildInputHashes
    }
    $trimBuildManifest | ConvertTo-Json -Depth 5 |
        Set-Content -LiteralPath (Join-Path $outputDirectory 'arm-trim-v4.6-manifest.json') -Encoding UTF8
    Write-Output "ELF: $elf"
    Write-Output "HEX: $hex"
    Write-Output "BIN: $bin"
} finally {
    Pop-Location
}

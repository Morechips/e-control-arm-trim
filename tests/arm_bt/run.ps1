$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
$armBtOriginalPath = $env:PATH
try {
    $env:PATH = (Split-Path (Get-Command gcc).Source) + ';' + $env:PATH
    New-Item -ItemType Directory -Force build | Out-Null
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic -Itests/arm_bt -ICore/Inc Core/Src/arm_tuner.c Core/Src/arm_trim.c Core/Src/arm_collision.c Core/Src/arm_trim_bluetooth.c Core/Src/arm_trim_bench.c Core/Src/arm_control.c Core/Src/arm_kinematics.c Core/Src/servo_pose.c Core/Src/servo_remote.c Core/Src/zlis2_driver.c Core/Src/bluetooth_driver.c Core/Src/serial_io.c Core/Src/pid_tuner.c tests/arm_bt/test_arm_tuner.c -lm -o build/test_arm_bt.exe
    if ($LASTEXITCODE -ne 0) { throw 'Arm Bluetooth test compile failed' }
    & ./build/test_arm_bt.exe
    if ($LASTEXITCODE -ne 0) { throw 'Arm Bluetooth tests failed' }
} finally { $env:PATH = $armBtOriginalPath; Pop-Location }



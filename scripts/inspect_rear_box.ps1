# Host calculations only; no OpenOCD, COM port or servo communication.
$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '..')
try {
    New-Item -ItemType Directory -Force build | Out-Null
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic -ICore/Inc Core/Src/arm_collision.c Core/Src/arm_trim.c Core/Src/arm_kinematics.c tests/arm_collision/inspect_rear_box.c -lm -o build/inspect_rear_box.exe
    if ($LASTEXITCODE -ne 0) { throw 'Rear box inspection compile failed' }
    & ./build/inspect_rear_box.exe
    if ($LASTEXITCODE -ne 0) { throw 'Rear box inspection failed' }
} finally { Pop-Location }

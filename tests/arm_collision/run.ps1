$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    New-Item -ItemType Directory -Force build | Out-Null
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic -ICore/Inc Core/Src/arm_collision.c Core/Src/arm_kinematics.c tests/arm_collision/test_arm_collision.c -lm -o build/test_arm_collision.exe
    if ($LASTEXITCODE -ne 0) { throw 'Arm collision compile failed' }
    & ./build/test_arm_collision.exe
    if ($LASTEXITCODE -ne 0) { throw 'Arm collision tests failed' }
} finally { Pop-Location }

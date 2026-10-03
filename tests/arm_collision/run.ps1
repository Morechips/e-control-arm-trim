$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    $buildDir = if ($env:CAR_TEST_BUILD_DIR) { $env:CAR_TEST_BUILD_DIR } else { 'build-local' }
    New-Item -ItemType Directory -Force $buildDir | Out-Null
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic -ICore/Inc Core/Src/arm_collision.c Core/Src/arm_trim.c Core/Src/arm_kinematics.c Core/Src/arm_trim_project.c tests/arm_collision/test_arm_collision.c -lm -o $buildDir/test_arm_collision.exe
    if ($LASTEXITCODE -ne 0) { throw 'Arm collision compile failed' }
    & (Join-Path $buildDir 'test_arm_collision.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Arm collision tests failed' }
} finally { Pop-Location }

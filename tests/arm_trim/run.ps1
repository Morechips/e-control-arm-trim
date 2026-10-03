$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
$trimOriginalPath = $env:PATH
try {
    $env:PATH = (Split-Path (Get-Command gcc).Source) + ';' + $env:PATH
    $buildDir = if ($env:CAR_TEST_BUILD_DIR) { $env:CAR_TEST_BUILD_DIR } else { 'build-local' }
    New-Item -ItemType Directory -Force $buildDir | Out-Null
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic -ICore/Inc Core/Src/arm_trim.c Core/Src/arm_collision.c Core/Src/arm_kinematics.c Core/Src/arm_trim_project.c tests/arm_trim/test_arm_trim.c -lm -o $buildDir/test_arm_trim.exe
    if ($LASTEXITCODE -ne 0) { throw 'Arm trim test compile failed' }
    & (Join-Path $buildDir 'test_arm_trim.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Arm trim tests failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic -ICore/Inc Core/Src/arm_trim.c Core/Src/arm_collision.c Core/Src/arm_kinematics.c Core/Src/arm_trim_project.c tests/arm_trim/test_arm_trim_async.c -lm -o $buildDir/test_arm_trim_async.exe
    if ($LASTEXITCODE -ne 0) { throw 'Arm trim async compile failed' }
    & (Join-Path $buildDir 'test_arm_trim_async.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Arm trim async tests failed' }
} finally { $env:PATH = $trimOriginalPath; Pop-Location }

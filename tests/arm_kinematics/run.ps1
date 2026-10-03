$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
$kinematicsOriginalPath = $env:PATH
try {
    $env:PATH = (Split-Path (Get-Command gcc).Source) + ';' + $env:PATH
    $buildDir = if ($env:CAR_TEST_BUILD_DIR) { $env:CAR_TEST_BUILD_DIR } else { 'build' }
    New-Item -ItemType Directory -Force $buildDir | Out-Null
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic -ICore/Inc Core/Src/arm_kinematics.c Core/Src/arm_kinematics_legacy_project.c tests/arm_kinematics/test_arm_kinematics.c -lm -o $buildDir/test_arm_kinematics.exe
    if ($LASTEXITCODE -ne 0) { throw 'Arm kinematics host test compile failed' }
    & (Join-Path $buildDir 'test_arm_kinematics.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Arm kinematics host tests failed' }
} finally {
    $env:PATH = $kinematicsOriginalPath
    Pop-Location
}


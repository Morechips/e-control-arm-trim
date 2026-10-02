$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
$kinematicsOriginalPath = $env:PATH
try {
    $env:PATH = (Split-Path (Get-Command gcc).Source) + ';' + $env:PATH
    New-Item -ItemType Directory -Force build | Out-Null
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic -ICore/Inc Core/Src/arm_kinematics.c tests/arm_kinematics/test_arm_kinematics.c -lm -o build/test_arm_kinematics.exe
    if ($LASTEXITCODE -ne 0) { throw 'Arm kinematics host test compile failed' }
    & ./build/test_arm_kinematics.exe
    if ($LASTEXITCODE -ne 0) { throw 'Arm kinematics host tests failed' }
} finally {
    $env:PATH = $kinematicsOriginalPath
    Pop-Location
}


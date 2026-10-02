$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
$trimOriginalPath = $env:PATH
try {
    $env:PATH = (Split-Path (Get-Command gcc).Source) + ';' + $env:PATH
    New-Item -ItemType Directory -Force build | Out-Null
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic -ICore/Inc Core/Src/arm_trim.c Core/Src/arm_collision.c Core/Src/arm_kinematics.c tests/arm_trim/test_arm_trim.c -lm -o build/test_arm_trim.exe
    if ($LASTEXITCODE -ne 0) { throw 'Arm trim test compile failed' }
    & ./build/test_arm_trim.exe
    if ($LASTEXITCODE -ne 0) { throw 'Arm trim tests failed' }
} finally { $env:PATH = $trimOriginalPath; Pop-Location }

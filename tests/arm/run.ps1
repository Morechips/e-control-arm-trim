$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
$armOriginalPath = $env:PATH
try {
    $env:PATH = (Split-Path (Get-Command gcc).Source) + ';' + $env:PATH
    New-Item -ItemType Directory -Force build | Out-Null
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic -Itests/arm -ICore/Inc Core/Src/arm_control.c Core/Src/zlis2_driver.c tests/arm/test_arm.c -o build/test_arm.exe
    if ($LASTEXITCODE -ne 0) { throw 'Arm host test compile failed' }
    & ./build/test_arm.exe
    if ($LASTEXITCODE -ne 0) { throw 'Arm host tests failed' }
} finally {
    $env:PATH = $armOriginalPath
    Pop-Location
}


$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
$zlis2OriginalPath = $env:PATH
try {
    $env:PATH = (Split-Path (Get-Command gcc).Source) + ';' + $env:PATH
    New-Item -ItemType Directory -Force build | Out-Null
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic -Itests/zlis2 -ICore/Inc Core/Src/zlis2_driver.c Core/Src/servo_pose.c tests/zlis2/test_zlis2.c -o build/test_zlis2.exe
    if ($LASTEXITCODE -ne 0) { throw 'ZLIS2 host test compile failed' }
    & ./build/test_zlis2.exe
    if ($LASTEXITCODE -ne 0) { throw 'ZLIS2 host tests failed' }
} finally {
    $env:PATH = $zlis2OriginalPath
    Pop-Location
}

$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
$visionOriginalPath = $env:PATH
try {
    $env:PATH = (Split-Path (Get-Command gcc).Source) + ';' + $env:PATH
    New-Item -ItemType Directory -Force build | Out-Null
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic -ICore/Inc Core/Src/vision_data.c tests/vision/test_vision_data.c -o build/test_vision_data.exe
    if ($LASTEXITCODE -ne 0) { throw 'Vision host test compile failed' }
    & ./build/test_vision_data.exe
    if ($LASTEXITCODE -ne 0) { throw 'Vision host tests failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic -ICore/Inc tests/vision/test_detect_data.c -o build/test_detect_data.exe
    if ($LASTEXITCODE -ne 0) { throw 'MaxiCam data test compile failed' }
    & ./build/test_detect_data.exe
    if ($LASTEXITCODE -ne 0) { throw 'MaxiCam data tests failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic -Itests/car -ICore/Inc Core/Src/maxicam.c tests/vision/test_maxicam.c -o build/test_maxicam.exe
    if ($LASTEXITCODE -ne 0) { throw 'MaxiCam UART test compile failed' }
    & ./build/test_maxicam.exe
    if ($LASTEXITCODE -ne 0) { throw 'MaxiCam UART tests failed' }
} finally {
    $env:PATH = $visionOriginalPath
    Pop-Location
}

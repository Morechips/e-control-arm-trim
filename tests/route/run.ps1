$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
$routeOriginalPath = $env:PATH
try {
    $env:PATH = (Split-Path (Get-Command gcc).Source) + ';' + $env:PATH
    New-Item -ItemType Directory -Force build | Out-Null
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic -Itests/car -ICore/Inc `
        Core/Src/action_fsm.c Core/Src/mission_fsm.c Core/Src/route_fsm.c Core/Src/maxicam.c Core/Src/laser.c tests/route/test_route_fsm.c `
        -o build/test_route_fsm.exe
    if ($LASTEXITCODE -ne 0) { throw 'Route FSM host test compile failed' }
    & ./build/test_route_fsm.exe
    if ($LASTEXITCODE -ne 0) { throw 'Route FSM host tests failed' }
} finally {
    $env:PATH = $routeOriginalPath
    Pop-Location
}

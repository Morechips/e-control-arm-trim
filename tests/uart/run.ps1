$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
$uartOriginalPath = $env:PATH
try {
    $env:PATH = (Split-Path (Get-Command gcc).Source) + ';' + $env:PATH
    $buildDir = if ($env:CAR_TEST_BUILD_DIR) { $env:CAR_TEST_BUILD_DIR } else { 'build' }
    New-Item -ItemType Directory -Force $buildDir | Out-Null
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic -Itests/uart -ICore/Inc Core/Src/uart_driver.c Core/Src/uart_tx_queue.c tests/uart/test_uart.c -o (Join-Path $buildDir 'test_uart.exe')
    if ($LASTEXITCODE -ne 0) { throw 'UART unit compile failed' }
    & (Join-Path $buildDir 'test_uart.exe')
    if ($LASTEXITCODE -ne 0) { throw 'UART unit tests failed' }
} finally { $env:PATH = $uartOriginalPath; Pop-Location }

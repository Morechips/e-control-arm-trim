$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    $compiler = (Get-Command gcc -ErrorAction Stop).Source
    $outputDirectory = if ($env:CAR_TEST_BUILD_DIR) { $env:CAR_TEST_BUILD_DIR } else { 'build-local' }
    New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null
    $executable = Join-Path $outputDirectory 'arm_trim_portable.exe'
    & $compiler -std=c11 -Wall -Wextra -Werror -pedantic -ICore/Inc Core/Src/arm_kinematics.c Core/Src/arm_collision.c Core/Src/arm_trim.c examples/arm_trim_portable/demo.c -lm -o $executable
    if ($LASTEXITCODE -ne 0) { throw 'Portable example compile failed' }
    & (Resolve-Path -LiteralPath $executable).Path
    if ($LASTEXITCODE -ne 0) { throw 'Portable example failed' }
} finally { Pop-Location }

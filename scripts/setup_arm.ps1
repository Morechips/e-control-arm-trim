param(
    [string]$Port,
    [int]$Baud = 115200,
    [string]$Config,
    [switch]$UseCurrentExample,
    [string]$PythonExecutable
)

$ErrorActionPreference = 'Stop'
$setupRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
if (-not $Config) { $Config = Join-Path $setupRoot 'configs\arm_installation.json' }
if (-not $PythonExecutable) {
    if (Test-Path -LiteralPath 'D:\Anaconda\python.exe') {
        $PythonExecutable = 'D:\Anaconda\python.exe'
    } else { $PythonExecutable = (Get-Command python -ErrorAction Stop).Source }
}
$setupArgs = @('-X', 'utf8', (Join-Path $PSScriptRoot 'arm_setup.py'), 'wizard', '--config', $Config, '--baud', $Baud)
if ($Port) { $setupArgs += @('--port', $Port) }
if ($UseCurrentExample) { $setupArgs += '--current-example' }
& $PythonExecutable @setupArgs
if ($LASTEXITCODE -ne 0 -and $LASTEXITCODE -ne 130) {
    throw "Arm setup exited with code $LASTEXITCODE"
}

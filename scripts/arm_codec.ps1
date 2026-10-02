param(
    [Parameter(Position = 0)]
    [string]$Value,
    [switch]$Decode,
    [switch]$CRLF,
    [switch]$Spaced,
    [switch]$NoTerminator
)

$ErrorActionPreference = 'Stop'

function ConvertTo-Hex([string]$Text) {
    $suffix = if ($NoTerminator) { '' } elseif ($CRLF) { "`r`n" } else { "`n" }
    $bytes = [Text.Encoding]::ASCII.GetBytes($Text + $suffix)
    $separator = if ($Spaced) { ' ' } else { '' }
    return [BitConverter]::ToString($bytes).Replace('-', $separator)
}

function ConvertFrom-Hex([string]$Hex) {
    $clean = $Hex -replace '&#x20;', ' '
    $clean = $clean -replace '(?i)0x', ''
    $clean = $clean -replace '[^0-9A-Fa-f]', ''
    if (($clean.Length % 2) -ne 0) { throw 'Hex digit count must be even.' }
    $bytes = for ($i = 0; $i -lt $clean.Length; $i += 2) {
        [Convert]::ToByte($clean.Substring($i, 2), 16)
    }
    return [Text.Encoding]::ASCII.GetString($bytes)
}

if ($Decode) {
    if ([string]::IsNullOrWhiteSpace($Value)) { $Value = Read-Host 'Hex' }
    ConvertFrom-Hex $Value
    exit 0
}
if (-not [string]::IsNullOrWhiteSpace($Value)) {
    ConvertTo-Hex $Value
    exit 0
}
Write-Host 'ARM text-to-hex mode. LF is appended automatically.'
Write-Host 'Enter an empty line to quit.'
while ($true) {
    $line = Read-Host 'Command'
    if ([string]::IsNullOrEmpty($line)) { break }
    ConvertTo-Hex $line
}

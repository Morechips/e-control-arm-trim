param(
    [switch]$SymbolsOnly,
    [string]$OpenOcdExecutable = 'D:\Tool\xpack-openocd-0.12.0-7\bin\openocd.exe',
    [string]$NmExecutable = '',
    [string]$GdbExecutable = '',
    [ValidateSet('firmware_direct', 'rollback_v4_5_20261002', 'rollback_v4_4_20261002', 'rollback_v4_3_20261002', 'rollback_v4_2_20261002', 'rollback_v4_1_20261002', 'rollback_v4_20261002', 'rollback_v3_1_20261001', 'rollback_v3_20261001')]
    [string]$FirmwareDirectory = 'firmware_direct'
)

$ErrorActionPreference = 'Stop'
$trimDiagRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$trimDiagElf = Join-Path (Join-Path $trimDiagRoot $FirmwareDirectory) 'stm32f407_bt_oled.elf'
$trimDiagHex = Join-Path (Join-Path $trimDiagRoot $FirmwareDirectory) 'stm32f407_bt_oled.hex'
$trimDiagNm = $NmExecutable
$trimDiagGdb = $GdbExecutable
if ([string]::IsNullOrWhiteSpace($trimDiagGdb)) {
    $trimDiagGdbCommand = Get-Command arm-none-eabi-gdb -ErrorAction SilentlyContinue
    if ($null -ne $trimDiagGdbCommand) { $trimDiagGdb = $trimDiagGdbCommand.Source }
    else { $trimDiagGdb = 'C:\ST\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin\arm-none-eabi-gdb.exe' }
}
if ([string]::IsNullOrWhiteSpace($trimDiagNm)) {
    $trimDiagNmCommand = Get-Command arm-none-eabi-nm -ErrorAction SilentlyContinue
    if ($null -ne $trimDiagNmCommand) { $trimDiagNm = $trimDiagNmCommand.Source }
    else { $trimDiagNm = 'C:\ST\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin\arm-none-eabi-nm.exe' }
}
foreach ($trimDiagPath in @($OpenOcdExecutable, $trimDiagElf, $trimDiagHex, $trimDiagNm, $trimDiagGdb)) {
    if (-not (Test-Path -LiteralPath $trimDiagPath -PathType Leaf)) {
        throw "Required file not found: $trimDiagPath"
    }
}

$trimDiagSymbols = @{}
$trimDiagSymbolLines = & $trimDiagNm -l --defined-only $trimDiagElf
if ($LASTEXITCODE -ne 0) { throw 'Cannot read firmware symbols.' }
foreach ($trimDiagLine in $trimDiagSymbolLines) {
    # Local symbols such as "sequence" occur in multiple modules.
    # Use DWARF source attribution instead of taking a same-name address.
    if ($trimDiagLine -match '^([0-9a-fA-F]+)\s+\w\s+(\S+)\s+.*[\\/]bluetooth_driver\.c:\d+$') {
        $trimDiagSymbols[$Matches[2]] = $Matches[1]
    }
}
$trimDiagFields = @(
    @{ Name='rx_byte_count'; Label='BT_RX_BYTES'; Width='mdw'; Count=1 },
    @{ Name='test_sequence'; Label='BT_TRIM_VALID_FRAMES'; Width='mdw'; Count=1 },
    @{ Name='simple_press_count'; Label='BT_TRIM_PRESSES'; Width='mdw'; Count=1 },
    @{ Name='invalid_frame_count'; Label='BT_INVALID_FRAMES'; Width='mdw'; Count=1 },
    @{ Name='sequence'; Label='BT_BASE_VALID_FRAMES'; Width='mdw'; Count=1 },
    @{ Name='bluetooth_rx_recoveries'; Label='BT_UART_RECOVERIES'; Width='mdw'; Count=1 },
    @{ Name='last_test_frame_length'; Label='BT_TRIM_LAST_LENGTH'; Width='mdb'; Count=1 },
    @{ Name='last_simple_action'; Label='BT_TRIM_LAST_PRESS_BITS'; Width='mdb'; Count=1 },
    @{ Name='last_traced_len'; Label='BT_RAW_LAST_LENGTH'; Width='mdb'; Count=1 },
    @{ Name='last_traced_bytes'; Label='BT_RAW_LAST_FRAME'; Width='mdb'; Count=41 }
)
$trimDiagIsService = [bool]($trimDiagSymbolLines -match '[\\/]arm_trim_service\.c:')
$trimDiagTypedFields = @(
    @{ Label='ARM_STATE'; Expr="'arm_trim_bluetooth.c'::trim.status.state"; Count=1 },
    @{ Label='ARM_ERROR'; Expr="'arm_trim_bluetooth.c'::trim.status.error"; Count=1 },
    @{ Label='ARM_REQUEST_ERROR'; Expr="'arm_trim_bluetooth.c'::last_request_result"; Count=1 },
    @{ Label='ARM_REFERENCE_VALID'; Expr="'arm_trim_bluetooth.c'::trim.status.reference_valid"; Count=1 },
    @{ Label='ARM_OWNER'; Expr="'arm_trim_bluetooth.c'::owns_motion"; Count=1 },
    @{ Label='ARM_JOGGING'; Expr="'arm_trim_bluetooth.c'::trim.status.jogging"; Count=1 },
    @{ Label='ARM_SEGMENT_INDEX'; Expr="'arm_trim_bluetooth.c'::trim.status.segment_index"; Count=1 },
    @{ Label='ARM_SEGMENT_COUNT'; Expr="'arm_trim_bluetooth.c'::trim.status.segment_count"; Count=1 },
    @{ Label='ARM_EST_P'; Expr="'arm_trim_bluetooth.c'::trim.status.estimated_position[0]"; Count=3 },
    @{ Label='ARM_TARGET_P'; Expr="'arm_trim_bluetooth.c'::trim.status.target_position[0]"; Count=3 },
    @{ Label='ARM_P0_LIMITS'; Expr="'arm_trim_bluetooth.c'::trim.config.calibration[0].min_position"; Count=2 },
    @{ Label='ARM_P1_LIMITS'; Expr="'arm_trim_bluetooth.c'::trim.config.calibration[1].min_position"; Count=2 },
    @{ Label='ARM_P2_LIMITS'; Expr="'arm_trim_bluetooth.c'::trim.config.calibration[2].min_position"; Count=2 },
    @{ Label='ARM_MODEL_MIN_BITS'; Expr="'arm_trim_bluetooth.c'::trim.status.model_min_mm"; Count=1 },
    @{ Label='ARM_MODEL_MAX_BITS'; Expr="'arm_trim_bluetooth.c'::trim.status.model_max_mm"; Count=1 },
    @{ Label='ARM_ENABLED_MIN_BITS'; Expr="'arm_trim_bluetooth.c'::trim.status.enabled_min_mm"; Count=1 },
    @{ Label='ARM_ENABLED_MAX_BITS'; Expr="'arm_trim_bluetooth.c'::trim.status.enabled_max_mm"; Count=1 },
    @{ Label='ARM_OFFSET_BITS'; Expr="'arm_trim_bluetooth.c'::trim.status.offset_mm"; Count=1 },
    @{ Label='ARM_TARGET_OFFSET_BITS'; Expr="'arm_trim_bluetooth.c'::trim.status.target_offset_mm"; Count=1 }
)
if ($trimDiagIsService) {
    $trimDiagTypedFields = @($trimDiagTypedFields | ForEach-Object {
        $field = $_.Clone()
        $field.Expr = $field.Expr.Replace("'arm_trim_bluetooth.c'::trim.", "'arm_trim_service.c'::service.trim.")
        $field.Expr = $field.Expr.Replace("'arm_trim_bluetooth.c'::last_request_result", "'arm_trim_service.c'::service.last_request")
        $field.Expr = $field.Expr.Replace("'arm_trim_bluetooth.c'::owns_motion", "'arm_trim_service.c'::service.owns")
        $field
    })
    $trimDiagTypedFields += @{ Label='ARM_SERVICE_STATE'; Expr="'arm_trim_service.c'::service.state"; Count=1 }
} elseif (-not ($trimDiagSymbolLines -match '\s+last_request_result\s')) {
    $trimDiagTypedFields = @($trimDiagTypedFields | Where-Object {
        $_.Label -ne 'ARM_REQUEST_ERROR' -and $_.Label -ne 'ARM_JOGGING'
    })
}
$trimDiagGdbCommands = @('set language c')
foreach ($trimDiagField in $trimDiagTypedFields) {
    $trimDiagGdbCommands += ('printf "ADDR_{0}=0x%lx\n", (unsigned long)&{1}' -f $trimDiagField.Label, $trimDiagField.Expr)
    $trimDiagGdbCommands += ('printf "SIZE_{0}=%lu\n", (unsigned long)sizeof({1})' -f $trimDiagField.Label, $trimDiagField.Expr)
}
$trimDiagGdbTemporary = Join-Path $env:TEMP ('arm-symbols-' + [Guid]::NewGuid().ToString('N') + '.gdb')
$trimDiagAddresses = @{}
$trimDiagWidths = @{}
try {
    Set-Content -LiteralPath $trimDiagGdbTemporary -Value $trimDiagGdbCommands -Encoding ASCII
    $trimDiagGdbLines = & $trimDiagGdb -batch -nx -q $trimDiagElf -x $trimDiagGdbTemporary
    if ($LASTEXITCODE -ne 0) { throw 'Cannot resolve typed arm diagnostics from the ELF.' }
    foreach ($trimDiagLine in $trimDiagGdbLines) {
        if ($trimDiagLine -match '^ADDR_(\w+)=(0x[0-9a-fA-F]+)$') { $trimDiagAddresses[$Matches[1]] = $Matches[2] }
        elseif ($trimDiagLine -match '^SIZE_(\w+)=(1|2|4)$') { $trimDiagWidths[$Matches[1]] = 8 * [int]$Matches[2] }
    }
} finally { Remove-Item -LiteralPath $trimDiagGdbTemporary -ErrorAction SilentlyContinue }
if ($SymbolsOnly) {
    foreach ($trimDiagField in $trimDiagTypedFields) {
        if (-not $trimDiagAddresses.ContainsKey($trimDiagField.Label) -or
            -not $trimDiagWidths.ContainsKey($trimDiagField.Label)) {
            throw ('Missing typed diagnostic: ' + $trimDiagField.Label)
        }
        Write-Output ('{0}={1} WIDTH={2} COUNT={3}' -f $trimDiagField.Label,
            $trimDiagAddresses[$trimDiagField.Label], $trimDiagWidths[$trimDiagField.Label], $trimDiagField.Count)
    }
    Write-Output 'SYMBOLS_ONLY_DONE; no hardware connection attempted.'
    exit 0
}
$trimDiagCommands = @('init', 'set diag_failed [catch {', 'halt',
    ('verify_image ' + $FirmwareDirectory + '/stm32f407_bt_oled.hex'), 'echo CURRENT_FIRMWARE_VERIFIED')
$trimDiagFields = @($trimDiagFields | Where-Object { $trimDiagSymbols.ContainsKey($_.Name) })
foreach ($trimDiagField in $trimDiagFields) {
    if (-not $trimDiagSymbols.ContainsKey($trimDiagField.Name)) {
        throw ('Required diagnostic symbol missing: ' + $trimDiagField.Name)
    }
    $trimDiagWidth = 32
    if ($trimDiagField.Width -eq 'mdb') { $trimDiagWidth = 8 }
    # mdw/mdb inside a sourced Tcl catch may return their text silently.
    # Explicitly echo the list returned by read_memory instead.
    $trimDiagCommands += ('echo {0}=[read_memory 0x{1} {2} {3}]' -f $trimDiagField.Label,
        $trimDiagSymbols[$trimDiagField.Name], $trimDiagWidth, $trimDiagField.Count)
}
foreach ($trimDiagField in $trimDiagTypedFields) {
    if (-not $trimDiagAddresses.ContainsKey($trimDiagField.Label) -or -not $trimDiagWidths.ContainsKey($trimDiagField.Label)) {
        throw ('Missing typed diagnostic: ' + $trimDiagField.Label)
    }
    $trimDiagCommands += ('echo {0}=[read_memory {1} {2} {3}]' -f $trimDiagField.Label,
        $trimDiagAddresses[$trimDiagField.Label], $trimDiagWidths[$trimDiagField.Label], $trimDiagField.Count)
}
$trimDiagCommands += @('} diag_message]', 'set resume_failed [catch {resume} resume_message]',
    'if {$resume_failed} {set diag_failed 1; set diag_message $resume_message}', 'if {$diag_failed} {',
    'echo BT_DIAGNOSTICS_FAILED', 'echo $diag_message', '} else {',
    'echo BT_DIAGNOSTICS_DONE', '}', 'shutdown')
$trimDiagTemporary = Join-Path $env:TEMP ('arm-bt-diag-' + [Guid]::NewGuid().ToString('N') + '.cfg')
Write-Output ('SHA256: ' + (Get-FileHash -LiteralPath $trimDiagHex -Algorithm SHA256).Hash)
Write-Output ('Reference directory: ' + $FirmwareDirectory)
Write-Output 'Read only; no flash erase/write, reset, or servo commands. Park the car.'
Write-Output 'Release wt and wait for settling; pause phone periodic TX before the halt, then restore TX after reading.'
Write-Output 'The MCU is briefly halted for image verification and RAM reads, then resumed.'
Write-Output 'Memory values below are hexadecimal. Read only the first BT_RAW_LAST_LENGTH bytes of the raw frame.'
Push-Location $trimDiagRoot
try {
    Set-Content -LiteralPath $trimDiagTemporary -Value $trimDiagCommands -Encoding ASCII
    $trimDiagNativePreference = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        $trimDiagOutput = & $OpenOcdExecutable -f '.\openocd\horco-cmsis-dap-stm32f407.cfg' -f $trimDiagTemporary 2>&1
        $trimDiagExit = $LASTEXITCODE
    } finally { $ErrorActionPreference = $trimDiagNativePreference }
    $trimDiagLines = @($trimDiagOutput | ForEach-Object { $_.ToString() })
    $trimDiagLines | Write-Output
    foreach ($trimDiagLine in $trimDiagLines) {
        if ($trimDiagLine -match '^(ARM_\w+)_BITS=(0x[0-9a-fA-F]+)$') {
            $trimDiagLabel = $Matches[1]
            $trimDiagBits = [Convert]::ToUInt32($Matches[2].Substring(2), 16)
            $trimDiagValue = [BitConverter]::ToSingle([BitConverter]::GetBytes($trimDiagBits), 0)
            Write-Output ($trimDiagLabel + '_MM=' + $trimDiagValue.ToString('F3', [Globalization.CultureInfo]::InvariantCulture))
        } elseif ($trimDiagLine -match '^(ARM_\w+)=(0x[0-9a-fA-F]+(?:\s+0x[0-9a-fA-F]+)*)$') {
            $trimDiagLabel = $Matches[1]
            $trimDiagDecimal = @($Matches[2] -split '\s+' | ForEach-Object { [Convert]::ToUInt32($_.Substring(2), 16) })
            Write-Output ($trimDiagLabel + '_DEC=' + ($trimDiagDecimal -join ','))
        }
    }
    if ($trimDiagExit -ne 0 -or $trimDiagLines -notcontains 'BT_DIAGNOSTICS_DONE') {
        throw 'Image verification or diagnostic read failed. No firmware was written.'
    }
} finally {
    Pop-Location
    Remove-Item -LiteralPath $trimDiagTemporary -ErrorAction SilentlyContinue
}

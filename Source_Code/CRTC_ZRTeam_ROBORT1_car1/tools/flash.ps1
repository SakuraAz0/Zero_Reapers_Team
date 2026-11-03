# ============================================================================
#  Build + flash this STM32 project with ST-LINK in HOT-PLUG mode.
#
#  Why hot-plug: the firmware enables the IWDG (independent watchdog). After a
#  system reset the IWDG keeps counting, so a "reset then attach" connection can
#  fail with "Unable to get core ID" when NRST is not wired to the ST-LINK.
#  Hot-plug attaches to the already running target (main loop feeds the IWDG),
#  which works reliably without the NRST wire.
#
#  Usage (from anywhere):
#      powershell -ExecutionPolicy Bypass -File tools\flash.ps1
#  Optional:
#      -BuildType Release        (default Debug)
#      -NoBuild                  (flash the existing firmware.hex only)
# ============================================================================
[CmdletBinding()]
param(
    [string]$BuildType = 'Debug',
    [switch]$NoBuild
)

$ErrorActionPreference = 'Stop'

$root    = Split-Path -Parent $PSScriptRoot
$tc      = 'S:\STM32CubeMX\STM32CubeCLT_1.22.0'
$cmake   = Join-Path $tc 'CMake\bin\cmake.exe'
$ninja   = Join-Path $tc 'Ninja\bin\ninja.exe'
$objcopy = Join-Path $tc 'GNU-tools-for-STM32\bin\arm-none-eabi-objcopy.exe'
$cli     = Join-Path $tc 'STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
$buildD  = Join-Path $root '_build_flash'
$elf     = Join-Path $buildD 'CRTC_ZRTeam_ROBORT1.elf'
$hex     = Join-Path $root 'firmware.hex'

foreach ($tool in @($cmake, $objcopy, $cli)) {
    if (-not (Test-Path $tool)) { throw "tool not found: $tool" }
}
$env:PATH = (Join-Path $tc 'GNU-tools-for-STM32\bin') + ';' +
            (Join-Path $tc 'CMake\bin') + ';' +
            (Join-Path $tc 'Ninja\bin') + ';' + $env:PATH

if ($NoBuild -and (Test-Path $hex)) {
    Write-Host '[1/3] skip build, reuse firmware.hex'
} else {
    Write-Host '[1/3] configure'
    & $cmake -S $root -B $buildD -G Ninja `
        "-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake" `
        "-DCMAKE_BUILD_TYPE=$BuildType"
    if ($LASTEXITCODE -ne 0) { throw 'cmake configure failed' }

    Write-Host '[2/3] build'
    & $cmake --build $buildD
    if ($LASTEXITCODE -ne 0) { throw 'build failed' }

    & $objcopy -O ihex $elf $hex
    if ($LASTEXITCODE -ne 0) { throw 'objcopy failed' }
}

Write-Host '[3/3] flash (hot-plug mode)'
$ok = $false
for ($i = 1; $i -le 3; $i++) {
    Write-Host "  attempt $i"
    & $cli -c port=SWD mode=hotplug -w $hex -v
    if ($LASTEXITCODE -eq 0) { $ok = $true; break }
    Write-Host '  retry...'
    Start-Sleep -Milliseconds 600
}
if (-not $ok) {
    throw 'flash failed after 3 attempts. Check SWD wiring / ST-LINK, or wire NRST and use connect-under-reset.'
}

& $cli -c port=SWD mode=hotplug -rst | Out-Null
Write-Host 'done: firmware flashed and target reset'

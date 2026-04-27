[CmdletBinding()]
param(
    [ValidateSet("Debug", "Release")]
    [string]$Config = "Debug",

    [string]$ProbeSn = ""
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Resolve-Path (Join-Path $scriptDir "..\..")
$buildScript = Join-Path $scriptDir "build.ps1"
$elfPath = Join-Path $repoRoot "targets\stm32g474re_agv\build\$Config\stm32g474re_agv.elf"

& $buildScript -Config $Config
if ($LASTEXITCODE -ne 0) {
    throw "Build failed with exit code $LASTEXITCODE."
}

$programmerCommand = Get-Command "STM32_Programmer_CLI.exe" -ErrorAction SilentlyContinue
if ($programmerCommand) {
    $programmer = $programmerCommand.Source
}
else {
    $programmerCandidates = @(
        "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
        "C:\ST\STM32CubeIDE_1.17.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.300.202508131133\tools\bin\STM32_Programmer_CLI.exe",
        "C:\ST\STM32CubeIDE_1.17.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.0.202409170845\tools\bin\STM32_Programmer_CLI.exe"
    )
    $programmer = $programmerCandidates | Where-Object { Test-Path $_ } | Select-Object -First 1
}

if (-not $programmer) {
    throw "STM32_Programmer_CLI.exe was not found. Install STM32CubeProgrammer or add it to PATH."
}

$connectArgs = @("port=SWD")
if ($ProbeSn) {
    $connectArgs += "sn=$ProbeSn"
}

$programArgs = @("-c") + $connectArgs + @("-w", $elfPath, "-v", "-rst")
& $programmer @programArgs
if ($LASTEXITCODE -ne 0) {
    throw "STM32CubeProgrammer failed with exit code $LASTEXITCODE."
}

Write-Host "Flashed $elfPath"

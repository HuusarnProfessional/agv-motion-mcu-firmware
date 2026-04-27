[CmdletBinding()]
param(
    [ValidateSet("Debug", "Release")]
    [string]$Config = "Debug"
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Resolve-Path (Join-Path $scriptDir "..\..")
$targetDir = Join-Path $repoRoot "targets\stm32g474re_agv"
$buildDir = Join-Path $targetDir "build\$Config"
$elfPath = Join-Path $buildDir "stm32g474re_agv.elf"

$armToolchainBin = "C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\14.2 rel1\bin"
if (Test-Path $armToolchainBin) {
    $env:PATH = "$armToolchainBin;$env:PATH"
}

Push-Location $targetDir
try {
    $cachePath = Join-Path $buildDir "CMakeCache.txt"
    $ninjaPath = Join-Path $buildDir "build.ninja"
    if ((-not (Test-Path $cachePath)) -or (-not (Test-Path $ninjaPath))) {
        & cmake --preset $Config
        if ($LASTEXITCODE -ne 0) {
            throw "CMake configure failed with exit code $LASTEXITCODE."
        }
    }

    & cmake --build --preset $Config --parallel
    if ($LASTEXITCODE -ne 0) {
        throw "CMake build failed with exit code $LASTEXITCODE."
    }
}
finally {
    Pop-Location
}

if (-not (Test-Path $elfPath)) {
    throw "Build completed, but ELF file was not found: $elfPath"
}

Write-Host "Built $elfPath"

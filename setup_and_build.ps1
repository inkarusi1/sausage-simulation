# Setup and build SausageRodEditorDemo
# Run from repo root: .\setup_and_build.ps1
#
# Optional params:
#   -Config   Debug | Release (default: Release)
#   -Rebuild  Delete build-vs2 and regenerate from scratch
param(
    [string]$Config = "Release",
    [switch]$Rebuild
)

$Root = $PSScriptRoot
$BuildDir = Join-Path $Root "build-vs2"

# ---- Find cmake ---------------------------------------------------------------
$CmakeCandidates = @(
    "C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe",
    "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe",
    "C:\Program Files\Microsoft Visual Studio\2022\Enterprise\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe",
    "C:\Program Files\CMake\bin\cmake.exe"
)
$Cmake = $null
foreach ($p in $CmakeCandidates) {
    if (Test-Path $p) { $Cmake = $p; break }
}
if (-not $Cmake) {
    $cmd = Get-Command cmake -ErrorAction SilentlyContinue
    if ($cmd) { $Cmake = $cmd.Source }
}
if (-not $Cmake) {
    Write-Error "cmake not found. Install Visual Studio 2022 with C++ workload (includes CMake)."
    exit 1
}
Write-Host "[cmake]   $Cmake"

# ---- Find MSBuild -------------------------------------------------------------
$MsbuildCandidates = @(
    "C:\Program Files\Microsoft Visual Studio\2022\Professional\MSBuild\Current\Bin\amd64\MSBuild.exe",
    "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\amd64\MSBuild.exe",
    "C:\Program Files\Microsoft Visual Studio\2022\Enterprise\MSBuild\Current\Bin\amd64\MSBuild.exe"
)
$MSBuild = $null
foreach ($p in $MsbuildCandidates) {
    if (Test-Path $p) { $MSBuild = $p; break }
}
if (-not $MSBuild) {
    $cmd = Get-Command MSBuild -ErrorAction SilentlyContinue
    if ($cmd) { $MSBuild = $cmd.Source }
}
if (-not $MSBuild) {
    Write-Error "MSBuild not found. Install Visual Studio 2022 with C++ Desktop workload."
    exit 1
}
Write-Host "[MSBuild] $MSBuild"

# ---- CMake configure (generate build-vs2/) ------------------------------------
if ($Rebuild -and (Test-Path $BuildDir)) {
    Write-Host "Removing existing build-vs2/ ..."
    Remove-Item $BuildDir -Recurse -Force
}

$SlnFile = Join-Path $BuildDir "PositionBasedDynamics.sln"
if (-not (Test-Path $SlnFile)) {
    Write-Host ""
    Write-Host "=== CMake configure (first time, ~1 min) ==="
    & $Cmake -S $Root -B $BuildDir -G "Visual Studio 17 2022" -A x64
    if ($LASTEXITCODE -ne 0) { Write-Error "CMake configure failed."; exit 1 }
} else {
    Write-Host "build-vs2/ already exists, skipping CMake. (Use -Rebuild to regenerate)"
}

# ---- MSBuild compile ----------------------------------------------------------
$Proj = Join-Path $BuildDir "Demos\SausageRodEditorDemo\SausageRodEditorDemo.vcxproj"
Write-Host ""
Write-Host "=== Building SausageRodEditorDemo ($Config x64) ==="
& $MSBuild $Proj /p:Configuration=$Config /p:Platform=x64 /m
if ($LASTEXITCODE -ne 0) { Write-Error "MSBuild failed."; exit 1 }

Write-Host ""
Write-Host "Build succeeded! Output: bin\SausageRodEditorDemo.exe"
Write-Host "To run: cd bin; .\SausageRodEditorDemo.exe"

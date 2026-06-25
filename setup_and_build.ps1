# 一键配置并构建 SausageRodEditorDemo
# 在仓库根目录以 PowerShell 运行: .\setup_and_build.ps1
#
# 可选参数:
#   -Config   Debug | Release (默认 Release)
#   -Rebuild  若 build-vs2 已存在，删除后重新生成（用于环境异常时）
param(
    [string]$Config = "Release",
    [switch]$Rebuild
)

$Root = $PSScriptRoot
$BuildDir = Join-Path $Root "build-vs2"

# ── 1. 找 cmake ──────────────────────────────────────────────────────────────
$CmakePaths = @(
    "C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe",
    "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe",
    "C:\Program Files\CMake\bin\cmake.exe"
)
$Cmake = $null
foreach ($p in $CmakePaths) { if (Test-Path $p) { $Cmake = $p; break } }
if (-not $Cmake) { $Cmake = (Get-Command cmake -ErrorAction SilentlyContinue)?.Source }
if (-not $Cmake) {
    Write-Error "找不到 cmake，请安装 CMake 或 Visual Studio 2022（含 CMake 组件）后重试。"
    exit 1
}
Write-Host "[cmake] $Cmake"

# ── 2. 找 MSBuild ─────────────────────────────────────────────────────────────
$MsbuildPaths = @(
    "C:\Program Files\Microsoft Visual Studio\2022\Professional\MSBuild\Current\Bin\amd64\MSBuild.exe",
    "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\amd64\MSBuild.exe",
    "C:\Program Files\Microsoft Visual Studio\2022\Enterprise\MSBuild\Current\Bin\amd64\MSBuild.exe"
)
$MSBuild = $null
foreach ($p in $MsbuildPaths) { if (Test-Path $p) { $MSBuild = $p; break } }
if (-not $MSBuild) { $MSBuild = (Get-Command MSBuild -ErrorAction SilentlyContinue)?.Source }
if (-not $MSBuild) {
    Write-Error "找不到 MSBuild，请安装 Visual Studio 2022（含 C++ 桌面开发工作负载）后重试。"
    exit 1
}
Write-Host "[MSBuild] $MSBuild"

# ── 3. CMake 配置（生成 build-vs2/） ─────────────────────────────────────────
if ($Rebuild -and (Test-Path $BuildDir)) {
    Write-Host "删除旧 build-vs2/ ..."
    Remove-Item $BuildDir -Recurse -Force
}

if (-not (Test-Path (Join-Path $BuildDir "PositionBasedDynamics.sln"))) {
    Write-Host "`n=== 运行 CMake 配置（首次约需 1 分钟）==="
    & $Cmake -S $Root -B $BuildDir -G "Visual Studio 17 2022" -A x64
    if ($LASTEXITCODE -ne 0) { Write-Error "CMake 配置失败"; exit 1 }
} else {
    Write-Host "build-vs2/ 已存在，跳过 CMake 配置。（若要重新生成请加 -Rebuild 参数）"
}

# ── 4. MSBuild 编译 ───────────────────────────────────────────────────────────
$Proj = Join-Path $BuildDir "Demos\SausageRodEditorDemo\SausageRodEditorDemo.vcxproj"
Write-Host "`n=== 编译 SausageRodEditorDemo ($Config) ==="
& $MSBuild $Proj /p:Configuration=$Config /p:Platform=x64 /m
if ($LASTEXITCODE -ne 0) { Write-Error "MSBuild 编译失败"; exit 1 }

Write-Host "`n构建成功！可执行文件：bin\SausageRodEditorDemo.exe"
Write-Host "运行方式：cd bin && .\SausageRodEditorDemo.exe"

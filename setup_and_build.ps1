# Configure and build the course demo from a fresh clone.
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File .\setup_and_build.ps1
#   powershell -ExecutionPolicy Bypass -File .\setup_and_build.ps1 -Rebuild
#   powershell -ExecutionPolicy Bypass -File .\setup_and_build.ps1 -Config Debug
#   powershell -ExecutionPolicy Bypass -File .\setup_and_build.ps1 -BuildDir build-local

[CmdletBinding()]
param(
    [ValidateSet("Debug", "Release", "RelWithDebInfo", "MinSizeRel")]
    [string]$Config = "Release",

    [Alias("Rebuild")]
    [switch]$Reconfigure,

    [string]$BuildDir = "build-vs2",

    [switch]$Clean,

    [switch]$Run
)

$ErrorActionPreference = "Stop"

$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
if (-not [System.IO.Path]::IsPathRooted($BuildDir)) {
    $BuildDir = Join-Path $Root $BuildDir
}
$TargetName = "SausageRodEditorDemo"
$ProjectFile = Join-Path $BuildDir "Demos\SausageRodEditorDemo\SausageRodEditorDemo.vcxproj"
$OutputExe = Join-Path $Root "bin\SausageRodEditorDemo.exe"

function Fail($Message) {
    Write-Host ""
    Write-Host "ERROR: $Message" -ForegroundColor Red
    exit 1
}

function Quote-Argument([string]$Argument) {
    if ($Argument -match '^[A-Za-z0-9_+=:,./\\-]+$') {
        return $Argument
    }
    return '"' + ($Argument -replace '"', '\"') + '"'
}

function Run-Command($Title, $File, [string[]]$Arguments) {
    Write-Host ""
    Write-Host "=== $Title ===" -ForegroundColor Cyan
    Write-Host "$File $($Arguments -join ' ')"

    $pathValue = [Environment]::GetEnvironmentVariable("Path", "Process")
    $quotedFile = Quote-Argument $File
    $quotedArgs = ($Arguments | ForEach-Object { Quote-Argument $_ }) -join " "
    $cmdLine = "set PATH=&& set `"Path=$pathValue`" && $quotedFile $quotedArgs"

    & $env:ComSpec /d /c $cmdLine
    if ($LASTEXITCODE -ne 0) {
        Fail "$Title failed with exit code $LASTEXITCODE."
    }
}

function Find-VsWhere {
    $candidates = @(
        (Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"),
        (Join-Path $env:ProgramFiles "Microsoft Visual Studio\Installer\vswhere.exe")
    )

    foreach ($candidate in $candidates) {
        if ($candidate -and (Test-Path -LiteralPath $candidate)) {
            return $candidate
        }
    }
    return $null
}

function Find-WithVsWhere($VsWhere, [string]$Requires, [string]$FindPath) {
    if (-not $VsWhere) {
        return $null
    }

    $result = & $VsWhere -latest -products * -requires $Requires -find $FindPath 2>$null |
        Where-Object { $_ -and (Test-Path -LiteralPath $_) } |
        Select-Object -First 1
    if ($result) {
        return $result
    }
    return $null
}

function Find-CommandPath([string]$Name) {
    $cmd = Get-Command $Name -ErrorAction SilentlyContinue
    if ($cmd) {
        return $cmd.Source
    }
    return $null
}

if (-not (Test-Path -LiteralPath (Join-Path $Root "CMakeLists.txt"))) {
    Fail "Run this script from inside the PositionBasedDynamics repository."
}

$VsWhere = Find-VsWhere

$CMake = Find-CommandPath "cmake"
if (-not $CMake) {
    $CMake = Find-WithVsWhere $VsWhere "Microsoft.VisualStudio.Component.VC.CMake.Project" "Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
}
if (-not $CMake) {
    Fail "CMake was not found. Install Visual Studio 2022 with the C++ CMake tools component, or install CMake and add it to PATH."
}

$MSBuild = Find-WithVsWhere $VsWhere "Microsoft.Component.MSBuild" "MSBuild\Current\Bin\amd64\MSBuild.exe"
if (-not $MSBuild) {
    $MSBuild = Find-CommandPath "MSBuild.exe"
}
if (-not $MSBuild) {
    Fail "MSBuild was not found. Install Visual Studio 2022 or Visual Studio Build Tools with the C++ desktop workload."
}

$HasBundledDiscregrid = Test-Path -LiteralPath (Join-Path $Root "extern\Discregrid\CMakeLists.txt")
$HasBundledGenericParameters = Test-Path -LiteralPath (Join-Path $Root "extern\GenericParameters\CMakeLists.txt")
$Git = Find-CommandPath "git"
if ((-not $HasBundledDiscregrid -or -not $HasBundledGenericParameters) -and -not $Git) {
    Fail "Git was not found. Bundled external dependency sources are missing, so CMake would need Git to download them."
}

Write-Host "Repository: $Root"
Write-Host "CMake:     $CMake"
Write-Host "MSBuild:   $MSBuild"
if ($Git) {
    Write-Host "Git:       $Git"
}
else {
    Write-Host "Git:       not required; bundled external dependency sources are present"
}

if ($Reconfigure -and (Test-Path -LiteralPath $BuildDir)) {
    $resolvedRoot = (Resolve-Path -LiteralPath $Root).Path
    $resolvedBuild = (Resolve-Path -LiteralPath $BuildDir).Path
    if (-not $resolvedBuild.StartsWith($resolvedRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
        Fail "Refusing to remove build directory outside the repository: $resolvedBuild"
    }
    Write-Host ""
    Write-Host "Removing build directory: $BuildDir" -ForegroundColor Yellow
    Remove-Item -LiteralPath $BuildDir -Recurse -Force
}

Run-Command "CMake configure" $CMake @(
    "-S", $Root,
    "-B", $BuildDir,
    "-G", "Visual Studio 17 2022",
    "-A", "x64",
    "-DUSE_PYTHON_BINDINGS=OFF"
)

if ($Clean -and (Test-Path -LiteralPath $ProjectFile)) {
    Run-Command "MSBuild clean $TargetName ($Config x64)" $MSBuild @(
        $ProjectFile,
        "/t:Clean",
        "/p:Configuration=$Config",
        "/p:Platform=x64"
    )
}

if (-not (Test-Path -LiteralPath $ProjectFile)) {
    Fail "Project file was not generated: $ProjectFile"
}

Run-Command "MSBuild build $TargetName ($Config x64)" $MSBuild @(
    $ProjectFile,
    "/p:Configuration=$Config",
    "/p:Platform=x64",
    "/m"
)

if (-not (Test-Path -LiteralPath $OutputExe)) {
    Fail "Build finished, but the expected executable was not found: $OutputExe"
}

Write-Host ""
Write-Host "Build succeeded." -ForegroundColor Green
Write-Host "Executable: $OutputExe"
Write-Host "Run it with:"
Write-Host "  cd `"$Root\bin`""
Write-Host "  .\SausageRodEditorDemo.exe"

if ($Run) {
    Write-Host ""
    Write-Host "Launching $OutputExe ..."
    Start-Process -FilePath $OutputExe -WorkingDirectory (Join-Path $Root "bin")
}

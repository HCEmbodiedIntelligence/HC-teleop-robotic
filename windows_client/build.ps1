param(
    [string]$Python = "$PSScriptRoot\..\.venv\Scripts\python.exe",
    [string]$Inno = "$PSScriptRoot\..\.deps\Inno\ISCC.exe",
    [string]$Output = "$PSScriptRoot\..\runtime\windows-release"
)
$ErrorActionPreference = 'Stop'
$rootDir = (Resolve-Path "$PSScriptRoot\..").Path
$outputDir = [IO.Path]::GetFullPath($Output)
$buildDir = Join-Path $rootDir 'runtime\windows-build'
New-Item -ItemType Directory -Path $outputDir,$buildDir -Force | Out-Null
Push-Location $rootDir
try {
    & $Python -m PyInstaller --noconfirm --clean --onefile --windowed --name HC-Teleop --paths $rootDir --workpath $buildDir --specpath $buildDir --distpath $outputDir "$PSScriptRoot\main.py"
    if ($LASTEXITCODE) { throw 'EXE build failed' }
    & $Inno "/DArtifacts=$outputDir" "$PSScriptRoot\setup.iss"
    if ($LASTEXITCODE) { throw 'Installer build failed' }
} finally { Pop-Location }

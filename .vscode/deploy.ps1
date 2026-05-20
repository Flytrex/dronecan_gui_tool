param(
    [string]$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path,
    [string]$Destination = 'G:\Shared drives\Engineering\Lab Tools\DroneCAN GUI Tool, Flytrex Version'
)

$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $RepoRoot

Write-Host "Running winbuild.bat in $RepoRoot"
& cmd.exe /c "winbuild.bat"
if ($LASTEXITCODE -ne 0) {
    Write-Error "winbuild.bat failed with exit code $LASTEXITCODE"
    exit $LASTEXITCODE
}

if (-not (Test-Path -LiteralPath $Destination)) {
    Write-Error "Destination not accessible: $Destination"
    exit 1
}

$msi = Get-ChildItem -Path 'dist' -Filter '*.msi' -File |
       Sort-Object LastWriteTime -Descending |
       Select-Object -First 1
if (-not $msi) {
    Write-Error 'No MSI found in dist\'
    exit 1
}

Write-Host "Copying $($msi.Name) to $Destination"
Copy-Item -LiteralPath $msi.FullName -Destination $Destination -Force
Write-Host 'Done.'

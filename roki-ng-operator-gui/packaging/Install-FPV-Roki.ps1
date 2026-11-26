# Установка для текущего пользователя, без прав администратора.
$ErrorActionPreference = 'Stop'
$Source = Join-Path $PSScriptRoot 'FPV Roki'
if (-not (Test-Path (Join-Path $Source 'FPV Roki.exe'))) { throw 'Extract the entire ZIP before installing' }
$Destination = Join-Path $env:LOCALAPPDATA 'Programs/FPV Roki'
New-Item -ItemType Directory -Force (Split-Path $Destination -Parent) | Out-Null
if (Test-Path $Destination) { Move-Item $Destination "$Destination.backup-$(Get-Date -Format yyyyMMdd-HHmmss)" }
Copy-Item -Recurse $Source $Destination
$Shell = New-Object -ComObject WScript.Shell
$Shortcut = $Shell.CreateShortcut((Join-Path ([Environment]::GetFolderPath('Programs')) 'FPV Roki.lnk'))
$Shortcut.TargetPath = Join-Path $Destination 'FPV Roki.exe'
$Shortcut.WorkingDirectory = $Destination
$Shortcut.Save()
Write-Host "Installed: $Destination"

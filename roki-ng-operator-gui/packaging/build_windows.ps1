# Запуск в Developer PowerShell Visual Studio, с активным conda-окружением.
param([switch]$NativeOnly, [switch]$PackageOnly)
$ErrorActionPreference = 'Stop'
Set-Location (Split-Path $PSScriptRoot -Parent)
function Checked([string]$Program, [string[]]$Arguments) {
    & $Program @Arguments
    if ($LASTEXITCODE -ne 0) { throw "$Program failed: $LASTEXITCODE" }
}
if (-not $env:CONDA_PREFIX) { throw 'Activate packaging/windows-environment.yml first' }
$Root = (Get-Location).Path
$Qt = Join-Path $env:CONDA_PREFIX 'Library'
$Install = Join-Path $Root '.deps/windows/install'
if (-not $PackageOnly) {
New-Item -ItemType Directory -Force '.deps/src/KDDockWidgets', '.deps/windows' | Out-Null
Checked 'tar' @('-xf','native/sources/KDDockWidgets-v2.4.1.tar.gz','-C','.deps/src/KDDockWidgets','--strip-components=1')
Checked 'cmake' @('-S','.deps/src/KDDockWidgets','-B','.deps/windows/build','-G','Ninja','-DCMAKE_BUILD_TYPE=Release',"-DCMAKE_PREFIX_PATH=$Qt",'-DKDDockWidgets_FRONTENDS=qtquick','-DKDDockWidgets_QML_MODULE=OFF','-DKDDockWidgets_EXAMPLES=OFF','-DKDDockWidgets_TESTS=OFF','-DKDDockWidgets_PYTHON_BINDINGS=OFF','-DKDDockWidgets_NO_SPDLOG=ON')
Checked 'cmake' @('--build','.deps/windows/build','--parallel','4')
Checked 'cmake' @('--install','.deps/windows/build','--prefix',$Install)
Checked 'cmake' @('-S','native/macos','-B','.deps/windows/bootstrap','-G','Ninja','-DCMAKE_BUILD_TYPE=Release',"-DCMAKE_PREFIX_PATH=$Install;$Qt")
Checked 'cmake' @('--build','.deps/windows/bootstrap')
}
$env:PATH = "$Install/bin;$Install/lib;$env:PATH"
Checked 'python' @('-c',"from pathlib import Path; from PySide6.QtCore import qVersion; Path('.deps/windows/qt-version.txt').write_text(qVersion())")
if ($NativeOnly) { exit 0 }
Checked (Join-Path $Root '..\Roki FPV.cmd') @('--check-gui')
Checked 'python' @('-m','unittest','discover','-s','tests')
Checked 'python' @('-m','PyInstaller','--noconfirm','packaging/fpv_roki.spec')
# Проверка без DLL из conda и Visual Studio. У CI нет гарантированного GPU.
$env:PATH = "$env:SystemRoot/System32;$env:SystemRoot"
# Режим QML задаёт само приложение; тест не должен маскировать его настройки.
Remove-Item Env:QT_QUICK_BACKEND, Env:QT_QUICK_CONTROLS_STYLE, Env:QV4_FORCE_INTERPRETER, Env:QML_DISABLE_DISK_CACHE -ErrorAction SilentlyContinue
$FailedTests = @()
foreach ($Test in @('--self-test-video','--self-test-qml-basic','--self-test-qml-controls','--self-test-qml-docking','--self-test-gui')) {
    $Argument = $Test
    Write-Host "RUN: $Test"
    $Process = Start-Process -FilePath 'dist/FPV Roki/FPV Roki.exe' -ArgumentList $Argument -PassThru
    if (-not $Process.WaitForExit(60000)) { $Process.Kill(); throw "Timeout: $Test" }
    if (Test-Path 'fpv-self-test.log') { Get-Content 'fpv-self-test.log' }
    if ($Process.ExitCode -ne 0) { $FailedTests += "$Test ($($Process.ExitCode))" }
}
if ($FailedTests.Count) { throw "Failed: $FailedTests" }
Copy-Item 'packaging/Install-FPV-Roki.ps1' 'dist/Install-FPV-Roki.ps1'
Compress-Archive -Path 'dist/FPV Roki','dist/Install-FPV-Roki.ps1' -DestinationPath 'dist/FPV-Roki-windows-x64.zip' -Force
$Hash = (Get-FileHash 'dist/FPV-Roki-windows-x64.zip' -Algorithm SHA256).Hash.ToLower()
Set-Content 'dist/FPV-Roki-windows-x64.zip.sha256' "$Hash  FPV-Roki-windows-x64.zip" -Encoding ascii

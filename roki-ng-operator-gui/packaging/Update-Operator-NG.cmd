@echo off
setlocal
set "APP=%~dp0FPV Roki\FPV Roki.exe"
if not exist "%APP%" set "APP=%LOCALAPPDATA%\Programs\FPV Roki\FPV Roki.exe"
if not exist "%APP%" (
 echo Extract the complete Roki ZIP or install FPV Roki first.
 pause
 exit /b 1
)
start "Operator NG Updater" "%APP%" --upstream-updater

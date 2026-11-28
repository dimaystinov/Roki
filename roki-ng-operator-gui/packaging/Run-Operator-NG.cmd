@echo off
setlocal
set "APP=%~dp0FPV Roki\FPV Roki.exe"
if not exist "%APP%" set "APP=%LOCALAPPDATA%\Programs\FPV Roki\FPV Roki.exe"
if not exist "%APP%" exit /b 1
start "Operator NG" "%APP%" --upstream-run

@echo off
setlocal
cd /d "%~dp0roki-ng-operator-gui" || exit /b 1
if exist "%CONDA_PREFIX%\python.exe" if /i "%CONDA_DEFAULT_ENV%"=="fpv-roki-build" goto run
if defined CONDA_EXE goto activate
where conda.exe >nul 2>&1 && set "CONDA_EXE=conda.exe"
if not defined CONDA_EXE if exist "%USERPROFILE%\miniforge3\Scripts\conda.exe" set "CONDA_EXE=%USERPROFILE%\miniforge3\Scripts\conda.exe"
if not defined CONDA_EXE (
  echo Open Miniforge Prompt and activate fpv-roki-build. See README.md.
  pause
  exit /b 1
)
:activate
"%CONDA_EXE%" run --no-capture-output -n fpv-roki-build python roki_operator.py %*
exit /b %errorlevel%
:run
"%CONDA_PREFIX%\python.exe" roki_operator.py %*
exit /b %errorlevel%

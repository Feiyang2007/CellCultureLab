@echo off
chcp 65001 >nul
setlocal
set "PY=D:\IsaacSim5\env\Scripts\python.exe"
if not exist "%PY%" (
  echo Isaac Sim Python not found: %PY%
  pause
  exit /b 1
)
pushd "%TEMP%"
"%PY%" "%~dp0scripts\open_scene.py" "%~dp0assets\scenes\biolab_main_v06.usda" %*
set "RESULT=%ERRORLEVEL%"
popd
if not "%RESULT%"=="0" pause
exit /b %RESULT%

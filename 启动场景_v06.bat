@echo off
chcp 65001 >nul
setlocal
set "PY=D:\isaacsim-5.1.0\python.bat"
if not exist "%PY%" (
  echo Isaac Sim Python not found: %PY%
  pause
  exit /b 1
)
pushd "%TEMP%"
call "%PY%" "%~dp0scripts\open_scene.py" "%~dp0assets\scenes\biolab_main_v06.usda" %*
set "RESULT=%ERRORLEVEL%"
popd
if not "%RESULT%"=="0" pause
exit /b %RESULT%

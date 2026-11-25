@echo off
chcp 65001 >nul
setlocal
set "PY=D:\IsaacSim5\env\Scripts\python.exe"
if not exist "%PY%" (
  echo 找不到 Isaac Sim 的 python：%PY%
  pause
  exit /b 1
)
pushd "%TEMP%"
"%PY%" "%~dp0scripts\open_scene.py" %*
set "RESULT=%ERRORLEVEL%"
popd
if not "%RESULT%"=="0" pause
exit /b %RESULT%

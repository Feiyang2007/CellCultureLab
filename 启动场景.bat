@echo off
chcp 65001 >nul
setlocal
rem ISAAC_PY 指到那台的 python 就能换机器用；不设则用本机 5.1。
set "PY=%ISAAC_PY%"
if "%PY%"=="" set "PY=D:\isaacsim-5.1.0\python.bat"
if not exist "%PY%" (
  echo 找不到 Isaac Sim 的 python：%PY%
  pause
  exit /b 1
)
pushd "%TEMP%"
call "%PY%" "%~dp0scripts\open_scene.py" %*
set "RESULT=%ERRORLEVEL%"
popd
if not "%RESULT%"=="0" pause
exit /b %RESULT%

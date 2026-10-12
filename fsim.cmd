@echo off
rem One preset selects the configure, build, test and runtime directories.
setlocal DisableDelayedExpansion
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\launch\fsim.ps1" %*
exit /b %errorlevel%

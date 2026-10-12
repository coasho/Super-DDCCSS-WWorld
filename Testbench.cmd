@echo off
rem With arguments, run synchronously and return the testbench's exit status.
rem A double-click keeps the console minimized until the GUI closes.
setlocal DisableDelayedExpansion
if "%~1"=="" (
    start "flightsim testbench - close the testbench window to stop" /min cmd /d /c ""%~dp0fsim.cmd" testbench"
    exit /b 0
)
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\launch\fsim.ps1" --testbench-entry %*
exit /b %errorlevel%

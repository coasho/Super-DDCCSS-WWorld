@echo off
rem Double-click to run the original native engagement in one fullscreen Viewer.
setlocal DisableDelayedExpansion
pushd "%~dp0"
if errorlevel 1 exit /b 1
echo Starting the F-16C / AIM-120C engagement. Close the Viewer or press Esc to stop.
call "%~dp0fsim.cmd" --preset ucrt64-release python "%~dp0tools\launch\engagement.py" --build "%~dp0build\ucrt64-release" --scene "%~dp0tools\validation\viewer_perf\engagement.json"
set "demoStatus=%errorlevel%"
echo.
if not "%demoStatus%"=="0" (
    echo Engagement failed with exit code %demoStatus%. Check the messages above.
    pause
)
popd
exit /b %demoStatus%

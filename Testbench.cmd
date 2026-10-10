@echo off
rem flightsim testbench - double-click this file (or the "flightsim testbench" shortcut on the desktop).
rem
rem Two windows open: the testbench (pick a scenario on the left, set its options, press Run) and the
rem viewer (watch the aircraft fly it). Close the testbench window to stop; the viewer closes with it.
rem
rem Options pass through, e.g.   Testbench.cmd --offline      (map tiles from the cache only, never fetched)
rem                              Testbench.cmd --run fa10.recovery
rem The program itself lives in testbench\ (see testbench\README.md).
setlocal
set "ROOT=%~dp0"
if not exist "%ROOT%build\ucrt64-release\python\fsim-python.cmd" (
    echo The testbench is not built yet. In this folder, run:
    echo.
    echo     fsim build
    echo.
    pause
    exit /b 1
)
rem (its own console, minimised: it closes by itself when the testbench window is closed)
start "flightsim testbench - close the testbench window to stop" /min cmd /c ""%ROOT%fsim.cmd" testbench %*"

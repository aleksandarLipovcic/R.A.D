@echo off
setlocal

REM ============================================================================
REM sync_pyd.bat
REM ----------------------------------------------------------------------------
REM MSBuild produces a .dll; a pybind11 extension module needs a .pyd
REM extension for Python's import machinery to pick it up on Windows.
REM MSBuild does not do that rename/copy for you -- it has to happen as a
REM separate step after every build, or Python keeps silently importing
REM whatever .pyd happened to be sitting there before (exactly what caused
REM the stale-debug-output issue: DroneBackend.dll was freshly rebuilt,
REM but DroneBackend.pyd was a day old).
REM
REM This script closes that gap. Wired into a project's Post-Build Event
REM (see the "SETUP" section at the bottom of this file), it runs
REM automatically after every build -- Debug or Release, DroneBackend or
REM DroneBackend_test -- with no manual step to forget.
REM
REM USAGE
REM   From a Post-Build Event (recommended, see SETUP below):
REM     sync_pyd.bat "$(TargetPath)" "$(OutDir)" "$(TargetName)"
REM
REM   By hand, with no arguments, using the defaults right below (edit
REM   these to match whichever build you want to sync if you ever need to
REM   run this manually instead of through the Post-Build Event):
REM     sync_pyd.bat
REM ============================================================================

set "DLL_PATH=%~1"
set "OUT_DIR=%~2"
set "TARGET_NAME=%~3"

REM ── Manual-run defaults ─────────────────────────────────────────────────
if "%DLL_PATH%"=="" set "DLL_PATH=%~dp0DroneBackend.dll"
if "%OUT_DIR%"=="" set "OUT_DIR=%~dp0"
if "%TARGET_NAME%"=="" set "TARGET_NAME=DroneBackend"

REM %OUT_DIR% comes from MSBuild's $(OutDir) with a trailing backslash
REM already included -- don't add another one, or the path just looks
REM wrong in the messages below (still works, but confusing to read).
set "PYD_PATH=%OUT_DIR%%TARGET_NAME%.pyd"

if not exist "%DLL_PATH%" (
    echo [sync_pyd] ERROR: "%DLL_PATH%" not found -- build produced nothing to sync.
    exit /b 1
)

echo [sync_pyd] %DLL_PATH% --^> %PYD_PATH%
copy /Y "%DLL_PATH%" "%PYD_PATH%" >nul

if errorlevel 1 (
    echo [sync_pyd] ERROR: copy failed. Most likely DroneCockpitUI.py ^(or any
    echo [sync_pyd]        other process^) is running and holding "%PYD_PATH%"
    echo [sync_pyd]        open -- close it and rebuild.
    exit /b 1
)

echo [sync_pyd] done.
exit /b 0

REM ============================================================================
REM SETUP -- do this once per project (DroneBackend, and DroneBackend_test
REM if that target also needs it):
REM
REM   1. Copy this file into the project folder, e.g. next to
REM      DroneBackend.vcxproj.
REM   2. In Visual Studio: right-click the project -> Properties ->
REM      Configuration Properties -> Build Events -> Post-Build Event.
REM   3. Set Command Line to:
REM        call "$(ProjectDir)sync_pyd.bat" "$(TargetPath)" "$(OutDir)" "$(TargetName)"
REM   4. Make sure "Configuration" at the top of that dialog is set to
REM      "All Configurations" so this runs for both Debug and Release, not
REM      just whichever one was selected when you added it.
REM   5. Apply, then rebuild once to confirm you see the
REM      "[sync_pyd] ... --> ... done." lines in the Output window.
REM
REM From then on, every build (Build or Rebuild, Debug or Release) leaves
REM a .pyd that is byte-for-byte the .dll that was just compiled -- no
REM separate rename step to forget.
REM ============================================================================

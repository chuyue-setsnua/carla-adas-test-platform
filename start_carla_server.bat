@echo off
echo ============================================
echo   Starting CARLA Server
echo ============================================
echo.
echo Launching Unreal Engine (first launch may be slow)...
echo.
start "" "E:\CARLA\CarlaUE4.exe"
timeout /t 8 /nobreak >nul
echo CARLA server started!
echo Now run run_demo.bat to test.
pause

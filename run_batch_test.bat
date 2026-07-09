@echo off
cd /d E:\CARLA
echo Starting CARLA Batch Test (80 combinations)...
echo This will take 15-25 minutes. Results saved to test_report\
echo.
C:\Users\23366\AppData\Local\Programs\Python\Python312\python.exe E:\CARLA\batch_test.py
echo.
echo ============================================
echo Batch test finished!
echo ============================================
pause

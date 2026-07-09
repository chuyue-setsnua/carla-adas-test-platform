@echo off
chcp 65001 >nul 2>&1
echo ============================================
echo   CARLA 0.9.16 一键安装脚本
echo ============================================
echo.

REM ===== 配置区域 =====
set CARLA_DIR=E:\CARLA\CARLA_0.9.16
set VENV_DIR=E:\CARLA\carla_env

REM ===== 第一步：检查CARLA是否已解压 =====
if not exist "%CARLA_DIR%\CarlaUE4.exe" (
    echo [!] 未检测到 CARLA 主程序。
    echo.
    echo     请先下载 CARLA 0.9.16 预编译包：
    echo.
    echo     浏览器打开 GitHub Releases 页面
    echo     https://github.com/carla-simulator/carla/releases/tag/0.9.16
    echo     下载 "CARLA_0.9.16.zip"
    echo.
    echo     下载后解压到 E:\CARLA\CARLA_0.9.16\ 目录
    echo     确保 E:\CARLA\CARLA_0.9.16\CarlaUE4.exe 存在
    echo.
    pause
    exit /b 1
)
echo [OK] CARLA 目录已检测到：%CARLA_DIR%

REM ===== 第二步：安装 CARLA Python API =====
echo.
echo [2/3] 安装 CARLA Python API...

set WHL_FILE=
for %%f in ("%CARLA_DIR%\PythonAPI\carla\dist\carla-*.whl") do (
    set WHL_FILE=%%f
)

if "%WHL_FILE%"=="" (
    echo [!] 未找到 CARLA wheel 文件
    echo     请检查 %CARLA_DIR%\PythonAPI\carla\dist\ 目录
    pause
    exit /b 1
)

echo     找到: %WHL_FILE%
"%VENV_DIR%\Scripts\pip.exe" install "%WHL_FILE%" --force-reinstall
if errorlevel 1 (
    echo [!] CARLA wheel 安装失败，可能是 Python 版本不匹配
    pause
    exit /b 1
)
echo [OK] CARLA Python API 安装成功

REM ===== 第三步：验证安装 =====
echo.
echo [3/3] 验证安装...
"%VENV_DIR%\Scripts\python.exe" -c "import carla; print('CARLA Python API 已就绪!')"
if errorlevel 1 (
    echo [!] 验证失败
    pause
    exit /b 1
)

echo.
echo ============================================
echo   CARLA 安装完成！
echo ============================================
echo   启动服务器：运行 start_carla_server.bat
echo   运行示例：  运行 run_demo.bat
echo.
pause

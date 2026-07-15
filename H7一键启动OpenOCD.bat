@echo off
chcp 65001 > nul
title STM32H7 DAPLink OpenOCD Launcher

REM ===================== Config =====================
REM OpenOCD root directory.
set "OPENOCD_ROOT=E:\BaiduSyncdisk\strong\OpenOCD\DevEnv\openocd-v0.12.0-i686-w64-mingw32"
REM DAPLink interface config.
set "DAPLINK_CFG=E:\BaiduSyncdisk\strong\OpenOCD\DevEnv\ozone_daplink.cfg"
REM Wait time after starting OpenOCD.
set "WAIT_TIME=2"
REM ==================================================

set "OPENOCD_EXE=%OPENOCD_ROOT%\bin\openocd.exe"
set "TARGET_CFG=%OPENOCD_ROOT%\share\openocd\scripts\target\stm32h7x.cfg"

if not exist "%OPENOCD_EXE%" (
    echo [ERROR] Cannot find OpenOCD:
    echo %OPENOCD_EXE%
    pause
    exit /b 1
)

if not exist "%DAPLINK_CFG%" (
    echo [ERROR] Cannot find DAPLink config:
    echo %DAPLINK_CFG%
    pause
    exit /b 1
)

if not exist "%TARGET_CFG%" (
    echo [ERROR] Cannot find STM32H7 target config:
    echo %TARGET_CFG%
    pause
    exit /b 1
)

echo Starting STM32H7 OpenOCD debug server...
echo.
echo Connection info:
echo - GDB port    : 3333
echo - Telnet port : 4444
echo - TCL port    : 6666
echo.
echo Keep the OpenOCD window open while debugging.
echo.

start "OpenOCD - STM32H7 Debug Server" cmd /k ""%OPENOCD_EXE%" -f "%DAPLINK_CFG%" -f "%TARGET_CFG%""

timeout /t %WAIT_TIME% /nobreak > nul

echo OpenOCD debug server has been started.
echo Use Ozone or GDB to connect to localhost:3333.
echo.
timeout /t 2 > nul
exit /b 0

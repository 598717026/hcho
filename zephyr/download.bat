@echo off
::读取path环境变量到自定义变量中
set PATH=%PATH%;"D:\Program Files (x86)\SEGGER\JLink_V512f"
set remain=%path%
set jlinkpath="JLink"
echo download hex file ...
echo.
echo %remain%| findstr %jlinkpath% >nul
if %errorlevel% equ 0 (
jlink.exe -autoconnect 1 -device nrf52832_xxaa -if swd -speed 2000 -commandfile download.jlink
) else (
echo please add path environment variables first
)

pause
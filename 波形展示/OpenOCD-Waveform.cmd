@echo off
setlocal
set "SCRIPT_DIR=%~dp0"
py -3.12 "%SCRIPT_DIR%openocd_wave.py" --elf "%SCRIPT_DIR%..\build\Debug\threenibian.elf" --autostart
if errorlevel 1 pause
endlocal

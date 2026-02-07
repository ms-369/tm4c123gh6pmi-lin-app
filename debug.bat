@echo off
echo Starting debugger...

start openocd -f board\ti_ek-tm4c123gxl.cfg



arm-none-eabi-gdb -ex "target extended-remote :3333" .\build\final.elf

taskkill /im openocd.exe /f

echo Debugger closed...
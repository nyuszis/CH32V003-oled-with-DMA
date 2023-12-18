# CH32V003 RISC-V oled test with DMA

ssd1306 OLED display draw C4 pin ADC value

## compile on windows

install GCC toolchain or CH32duino

- CH32duino add to path 
    set PATH=%LOCALAPPDATA%\Arduino15\packages\WCH\tools\riscv-none-embed-gcc\8.2.0\bin;%PATH%
    and change ford.bat to set COMPILERPREFIX=riscv-none-embed-

upload to device with minichlink.exe 


@SET PRGNAME=test_ssd1306_oled
@SET COMPILERPREFIX=riscv64-unknown-elf-
@REM @SET COMPILERPREFIX=riscv-none-embed-
@if "%1"=="-c" (
    del %PRGNAME%.elf %PRGNAME%.bin %PRGNAME%.hex %PRGNAME%.lst %PRGNAME%.map
    goto end
) 
@%COMPILERPREFIX%gcc -o %PRGNAME%.elf ch32v003fun/ch32v003fun.c %PRGNAME%.c -g -Os -flto -ffunction-sections -static-libgcc -march=rv32ec -mabi=ilp32e -Ich32v003fun -nostdlib -I. -Wall  -T ch32v003fun/ch32v003fun.ld -Wl,--gc-sections -Lch32v003fun -lgcc
@%COMPILERPREFIX%objdump -S %PRGNAME%.elf > %PRGNAME%.lst
@%COMPILERPREFIX%objdump -t %PRGNAME%.elf > %PRGNAME%.map
@%COMPILERPREFIX%objcopy -O binary %PRGNAME%.elf %PRGNAME%.bin
@%COMPILERPREFIX%objcopy -O ihex %PRGNAME%.elf %PRGNAME%.hex
@%COMPILERPREFIX%size -d %PRGNAME%.elf | awk "/[0-9]/ {print ""size : "" $1 + $2 "" bytes""}"
@if "%1"=="-f" (
@..\minichlink -w %PRGNAME%.bin flash -b
)
:end


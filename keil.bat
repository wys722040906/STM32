@echo off

rem 清理对象文件
del /s /q *.o

rem 清理中间文件
del /s /q *.d

rem 清理编译生成的可执行文件
del /s /q *.elf

rem 清理编译生成的二进制文件
del /s /q *.bin
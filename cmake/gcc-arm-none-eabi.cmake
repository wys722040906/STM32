# 设置系统名称为 Generic，适用于没有特定系统的项目
set(CMAKE_SYSTEM_NAME               Generic)
# 设置处理器架构为 ARM
set(CMAKE_SYSTEM_PROCESSOR          arm)

# 强制指定编译器
set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)
# 指定编译器类型为 GNU
set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# 工具链前缀，必须包含在环境变量的路径中
set(TOOLCHAIN_PREFIX                arm-none-eabi-)

# 设置 C/C++ 编译器及相关工具
set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}gcc)  # C 编译器
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})     # 汇编编译器（使用 C 编译器）
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PREFIX}g++)  # C++ 编译器
set(CMAKE_LINKER                    ${TOOLCHAIN_PREFIX}g++)  # 链接器
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy)  # 对象拷贝工具
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size)     # 显示二进制大小的工具

# 设置可执行文件后缀为 .elf
set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

# 设置尝试编译的目标类型为静态库
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU 相关的编译标志
set(TARGET_FLAGS "-mcpu=cortex-m3 ")  # 设置目标为 Cortex-M3

# 将目标标志添加到 C 编译标志中
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
# 添加一些默认的 GCC 警告和优化选项
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wpedantic -Wno-unused-parameter -fdata-sections -ffunction-sections")

# 根据构建类型设置不同的优化和调试选项
if(CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g3")  # Debug模式下不优化，增加调试信息
endif()
if(CMAKE_BUILD_TYPE MATCHES Release)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Os -g0")  # Release模式下优化，减少大小，不增加调试信息
endif()

# 设置汇编编译标志
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
# 设置 C++ 编译标志，禁用 RTTI 和异常处理
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics -Wno-unused-parameter")

# 设置链接时的 C 编译标志
set(CMAKE_C_LINK_FLAGS "${TARGET_FLAGS}")  # 目标标志
# 链接脚本文件
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32F103C8Tx_FLASH.ld\"")
# 使用 nano.specs 来减小库的大小
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} --specs=nano.specs -Wl,--gc-sections -u _printf_float")
# 生成链接映射文件，启用垃圾回收选项
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
# 启动和结束分组链接，链接 libc 和 libm
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lc -lm -Wl,--end-group")
# 打印内存使用情况
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--print-memory-usage")

# C++ 链接标志，增加标准库的链接
set(CMAKE_CXX_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lstdc++ -lsupc++ -Wl,--end-group")

#预安装
opocd cmake mingw|msys2 gcc-arm-none-eabi-10.3-2021.10-win32.exe ninja-win

#修改jlink.cfg(D:\Program Files\OpenOCD-20240820-0.12.0\share\openocd\scripts\interface)
adapter driver jlink
transport select swd
#板子选型(D:\Program Files\OpenOCD-20240820-0.12.0\share\openocd\scripts\target)

#jlink安装+UsbDriverTool-2.1切换模式
----------------------------------------------------------
#运行
cmake -B build -G "Ninja"
ninja -C build
切换工程文件名
-----------------------------------------------------------
#防优化
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O0")
set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -O0")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O0")
#printf 浮点数  ---bug
# # 设置编译器选项
# target_link_options(${CMAKE_PROJECT_NAME} PRIVATE 
#     -specs=nano.specs 
#     -specs=nosys.specs 
#     -u _printf_float
#     -u _scanf_float
# )
# add_link_options(-specs=nano.specs -specs=nosys.specs -u _printf_float)
add_definitions(-specs=nosys.specs -specs=nano.specs -u _printf_float )


# Link directories setup
#指定库搜索路径
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable 
# 添加自定义源码
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CMAKE_SOURCE_DIR}/My_hardware/Src/retarget.c
)

# Add include paths
# 添加头文件路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_SOURCE_DIR}/My_hardware/Inc
)

# Add project symbols (macros)
#添加宏
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
#链接源文件依赖的目标库
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)


串口：重写/core/src/syscall.c
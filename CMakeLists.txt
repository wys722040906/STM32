cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O0")
set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -O0")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O0")

# Set the project name
set(CMAKE_PROJECT_NAME STM32_Template)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

add_custom_target(Flash
    COMMAND JLinkExe -if SWD -CommanderScript ${CMAKE_SOURCE_DIR}/STM32.JLinkScripts
    DEPENDS ${CMAKE_BINARY_DIR}/STM32_Template.elf  # 确保这里的目标名称是正确的
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/  # 可选，指定工作目录
)
# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CMAKE_SOURCE_DIR}/MyCode/lib/Src/CRC.c
    ${CMAKE_SOURCE_DIR}/MyCode/lib/Src/Serial_Analysis.c
    # ${CMAKE_SOURCE_DIR}/MyCode/lib/Src/file1.c
    # ${CMAKE_SOURCE_DIR}/MyCode/lib/Src/file2.c
    # ${CMAKE_SOURCE_DIR}/MyCode/task/Src/task1.c
    # ${CMAKE_SOURCE_DIR}/MyCode/task/Src/task2.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_SOURCE_DIR}/MyCode/lib/Inc
    # ${CMAKE_SOURCE_DIR}/MyCode/task/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# 设置目标为你的 STM32 HAL 工程的目标名称
# target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE 
#     -u 
#     _printf_float
#     -u 
#     _scanf_float
# )
# target_link_options(${CMAKE_PROJECT_NAME} PRIVATE 
#     -u 
#     _printf_float
#     -u 
#     _scanf_float
# )

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    -u _printf_float  # 链接浮点数支持
    # Add user defined libraries
)


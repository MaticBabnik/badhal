cmake_minimum_required(VERSION 3.14)

# setup compiler before project()
set(TOOL_PREFIX "arm-none-eabi-" CACHE STRING "point to $dir/arm-none-eabi- to if your compiler isnt in path")
set(CMAKE_C_COMPILER "${TOOL_PREFIX}gcc")
set(CMAKE_CXX_COMPILER "${TOOL_PREFIX}g++")
set(CMAKE_LINKER "${TOOL_PREFIX}ld")
set(OBJCOPY "${TOOL_PREFIX}objcopy")
set(OBJDUMP "${TOOL_PREFIX}objdump")
set(SIZE "${TOOL_PREFIX}size")

set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

project(MyProject C CXX ASM)

file(GLOB_RECURSE srcs "${CMAKE_SOURCE_DIR}/src/*.s" "${CMAKE_SOURCE_DIR}/src/*.c")

# Replace .s extension with .o for object files
foreach(source ${srcs})
    string(REGEX REPLACE "\\.[sS]$" ".o" obj ${source})
    list(APPEND OBJS ${obj})
    endforeach()

include_directories(${CMAKE_SOURCE_DIR})


set(LINK_MODE "FLASH" CACHE STRING "Store to FLASH or RAM?")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m7 -mthumb -g3 -O3 -flto -gdwarf -fno-common -Wall -pedantic -Wsign-compare -Wcast-align -Wconversion -fsingle-precision-constant -ffunction-sections -fdata-sections")

# Shouldn't affect projects that don't use FP at all
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfloat-abi=hard -mfpu=fpv5-d16") 

# Linker
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/linker/STM32H750XBHX_${LINK_MODE}.ld)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -mcpu=cortex-m7 -march=armv7e-m -nostartfiles --specs=nosys.specs -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/Main.map")

add_executable(starter ${srcs})


set_target_properties(starter PROPERTIES LINK_FLAGS "-T${LINKER_SCRIPT}")

add_custom_target(size
    COMMAND ${SIZE} ${CMAKE_CURRENT_BINARY_DIR}/starter
    DEPENDS starter
)

add_custom_target(binary
    COMMAND ${OBJCOPY} -O binary ${CMAKE_CURRENT_BINARY_DIR}/starter ${CMAKE_CURRENT_BINARY_DIR}/starter.bin
    DEPENDS starter
)

add_custom_target(burn
    COMMAND st-flash write ${CMAKE_CURRENT_BINARY_DIR}/starter.bin 0x8000000
    DEPENDS binary
)
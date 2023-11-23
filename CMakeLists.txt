cmake_minimum_required(VERSION 3.14)

# setup compiler before project()
set(TOOL_PREFIX "arm-none-eabi-" CACHE STRING "point to $dir/arm-none-eabi- to if your compiler isnt in path")
set(CMAKE_C_COMPILER "${TOOL_PREFIX}gcc")
set(CMAKE_CXX_COMPILER "${TOOL_PREFIX}g++")
set(CMAKE_LINKER "${TOOL_PREFIX}ld")
set(OBJCOPY "${TOOL_PREFIX}objcopy")
set(OBJDUMP "${TOOL_PREFIX}objdump")
set(SIZE "${TOOL_PREFIX}size")
set(DBG "${TOOL_PREFIX}gdb")

set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

project(MyProject C CXX ASM)

file(GLOB srcs "${CMAKE_SOURCE_DIR}/src/*.s")

# Replace .s extension with .o for object files
foreach(source ${srcs})
    string(REGEX REPLACE "\\.[sS]$" ".o" obj ${source})
    list(APPEND OBJS ${obj})
    endforeach()

include_directories(${CMAKE_SOURCE_DIR})

set(LINK_MODE "FLASH" CACHE STRING "Store to FLASH or RAM?")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m7 -marm -O2 -g3 -gdwarf -fno-common -Wall -pedantic -Wsign-compare -Wcast-align -Wconversion -fsingle-precision-constant -ffunction-sections -fdata-sections")

# Choose the relevant FPU option
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfloat-abi=soft") # No FP
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfloat-abi=softfp -mfpu=fpv4-sp-d16") # Soft FP
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfloat-abi=hard -mfpu=fpv4-sp-d16") # Hard FP

# Linker
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/linker/STM32H750XBHX_${LINK_MODE}.ld)  # For storing program in FLASH
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -march=armv7e-m -nostartfiles --specs=nosys.specs -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/Main.map")



# Define the executable target
add_executable(starter ${srcs})

# Set the linker script for the executable
set_target_properties(starter PROPERTIES LINK_FLAGS "-T${LINKER_SCRIPT}")


# Build the "build" target
add_custom_target(build
    DEPENDS starter
)

# Build the "size" target
add_custom_target(size
    COMMAND ${SIZE} ${CMAKE_CURRENT_BINARY_DIR}/starter
    DEPENDS starter
)

# Build the "disass" target
add_custom_target(disass
    COMMAND ${OBJDUMP} -d ${CMAKE_CURRENT_BINARY_DIR}/starter
    DEPENDS starter
)

# Build the "disass-all" target
add_custom_target(disass-all
    COMMAND ${OBJDUMP} -D ${CMAKE_CURRENT_BINARY_DIR}/starter
    DEPENDS starter
)

# Build the "debug" target
add_custom_target(debug
    COMMAND ${DBG} --eval-command="target extended-remote :4242" ${CMAKE_CURRENT_BINARY_DIR}/starter
    DEPENDS starter
)

# Build the "ocd" target
add_custom_target(ocd
    COMMAND ${DBG} --eval-command="target extended-remote :3333" --eval-command="monitor tpiu config internal swo.log uart off 16000000 1600000" ${CMAKE_CURRENT_BINARY_DIR}/starter
    DEPENDS starter
)

add_custom_target(binary
    COMMAND ${OBJCOPY} -O binary ${CMAKE_CURRENT_BINARY_DIR}/starter ${CMAKE_CURRENT_BINARY_DIR}/starter.bin
    DEPENDS starter
)

# Build the "burn" target
add_custom_target(burn
    COMMAND st-flash write ${CMAKE_CURRENT_BINARY_DIR}/starter.bin 0x8000000
    DEPENDS binary
)




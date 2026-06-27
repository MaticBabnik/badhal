cmake_minimum_required(VERSION 3.24)

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

set(GCC_PREFIX "arm-none-eabi-" CACHE STRING "Path and prefix for GCC and binutils")
set(CLANG_PATH "" CACHE STRING "Path for Clang and llvm tools")

# this one is for AUR "baremetal-compiler-rt" as of now (see commit date :) )
set(CLANG_RTLIB_PATH 
    "/usr/lib/clang/22/lib/baremetal/libclang_rt.builtins-armv7em.a" 
    CACHE STRING
    "Path to libclang_rt.builtins-armv7em.a"
)

option(USE_CLANG "Build with clang instead of gcc" OFF)

if (USE_CLANG)
    set(CMAKE_C_COMPILER "${CLANG_PATH}clang")
    set(CMAKE_LINKER "${CLANG_PATH}clang")
    set(OBJCOPY "${CLANG_PATH}llvm-objcopy")
    set(OBJDUMP "${CLANG_PATH}llvm-objdump")
    set(SIZE "${CLANG_PATH}llvm-size")
else()
    set(CMAKE_C_COMPILER "${GCC_PREFIX}gcc")
    set(CMAKE_LINKER "${GCC_PREFIX}ld")
    set(OBJCOPY "${GCC_PREFIX}objcopy")
    set(OBJDUMP "${GCC_PREFIX}objdump")
    set(SIZE "${GCC_PREFIX}size")
endif()

project(BadHAL C ASM)

include_directories(${CMAKE_SOURCE_DIR}/src)
file(GLOB_RECURSE srcs "${CMAKE_SOURCE_DIR}/src/*.c" )

set(LINKER_SCRIPT_NAME "badhal_flash.ld" CACHE STRING "Linker script filename, looked up in linker/")

set(CPU_FLAGS 
    -mthumb 
    -mcpu=cortex-m7
    -mfpu=fpv5-d16 
    -mfloat-abi=hard
    -march=armv7e-m+fp.dp
    -mtune=cortex-m7
)

set(OPT_FLAGS
    -g3
    -O3
    -flto
    -gdwarf
)

set(BADHAL_FLAGS
    -ffreestanding
    -fno-builtin
    -nostdlib
    -fno-common
    -ffunction-sections
    -fdata-sections    
)

set(BADHAL_LINKER_FLAGS
    -nostdlib
    -nostartfiles
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/Main.map
)

if (USE_CLANG)
    list(PREPEND CPU_FLAGS --target=armv7em-none-eabi)
    list(APPEND BADHAL_FLAGS -Wdouble-promotion)
    list(APPEND BADHAL_LINKER_FLAGS -fuse-ld=lld "${CLANG_RTLIB_PATH}")
else()
    list(APPEND BADHAL_FLAGS -fsingle-precision-constant)
    list(APPEND BADHAL_LINKER_FLAGS --specs=nosys.specs)
endif()

string(REPLACE ";" " " C_FLAGS_STR "${CPU_FLAGS} ${OPT_FLAGS} ${BADHAL_FLAGS}")
string(REPLACE ";" " " L_FLAGS_STR "${CPU_FLAGS} ${OPT_FLAGS} ${BADHAL_LINKER_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C_FLAGS_STR}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${L_FLAGS_STR}")


message(STATUS "Using compiler: ${CMAKE_C_COMPILER}")
message(STATUS "Using linker: ${CMAKE_LINKER}")
message(STATUS "Flags\nC: ${CMAKE_C_FLAGS}\nLinker: ${CMAKE_EXE_LINKER_FLAGS}")

# Linker
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/linker/${LINKER_SCRIPT_NAME})

add_executable(badhal ${srcs})

set_target_properties(badhal PROPERTIES LINK_FLAGS "-T${LINKER_SCRIPT}")

add_custom_target(size
    COMMAND ${SIZE} ${CMAKE_CURRENT_BINARY_DIR}/badhal
    DEPENDS badhal
)

add_custom_target(binary
    COMMAND ${OBJCOPY} -O binary ${CMAKE_CURRENT_BINARY_DIR}/badhal ${CMAKE_CURRENT_BINARY_DIR}/badhal.bin
    DEPENDS badhal
)

add_custom_target(burn
    COMMAND st-flash write ${CMAKE_CURRENT_BINARY_DIR}/badhal.bin 0x8000000
    DEPENDS binary
)

add_custom_target(burn-reset
    COMMAND st-flash --connect-under-reset write ${CMAKE_CURRENT_BINARY_DIR}/badhal.bin 0x8000000
    DEPENDS binary
)
cmake_minimum_required(VERSION 3.28)
project(badfs-tools VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(badfs-tools
    src/main.cpp
)

target_include_directories(badfs-tools PRIVATE include)

target_compile_options(badfs-tools PRIVATE -Wall -Wextra -Wpedantic)

cmake_minimum_required(VERSION 3.10)
project(posix_drone_sim C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# default if nothing specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

add_subdirectory(headers)
add_subdirectory(src)

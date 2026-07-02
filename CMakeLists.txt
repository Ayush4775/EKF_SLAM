cmake_minimum_required(VERSION 3.16)
project(ekf_slam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(ekf_slam
    src/main.cpp
    src/ekf_slam.cpp
)

target_compile_options(ekf_slam PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

target_include_directories(ekf_slam PRIVATE
    /opt/homebrew/include/eigen3
    include
)
cmake_minimum_required(VERSION 3.1)

project(pitranger)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(libs)

add_library(libpitranger
    src/sensors/pr_imu.cpp
    src/actuators/pr_ptu.cpp
    src/actuators/pr_wheel.cpp
)
target_link_libraries(libpitranger
    xsdevice
    fmt::fmt
    eigen
    roboteq
    libserial_static
    stdc++fs
    cxxopts
)
target_include_directories(libpitranger
    PUBLIC
    include/
    src/
    src/actuators
    src/sensors
    src/utils
)

add_subdirectory(tools)

cmake_minimum_required(VERSION 3.1)

project(pitranger)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(libs)

add_library(libpitranger
    src/pr_imu.cpp
    src/pr_motor.cpp
    src/actuators/pr_ptu.cpp
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
)

add_subdirectory(tools)


add_executable(teleop
    src/teleop.cpp
)
target_link_libraries(teleop
    libpitranger
)

add_executable(mstop
    src/mstop.cpp
)
target_link_libraries(mstop
    libpitranger
)

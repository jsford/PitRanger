cmake_minimum_required(VERSION 3.1)

project(pitranger)
set(CMAKE_CXX_STANDARD 17)

# Build bundled libraries.
add_subdirectory(libs)

# Find Spinnaker Camera API for PitCam.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake" CACHE STRING "Modules for CMake" FORCE)
find_package(Spinnaker REQUIRED)

add_library(libpitranger
    src/sensors/pr_imu.cpp
    src/sensors/pr_pitcam.cpp
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
    ${Spinnaker_LIBRARIES}
)
target_include_directories(libpitranger
    PUBLIC
    include/
    PUBLIC
    src/
    src/actuators
    src/sensors
    src/utils
    ${Spinnaker_INCLUDE_DIRS}
)

# Build tools for debugging HW/SW on the robot.
add_subdirectory(tools)

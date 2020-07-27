cmake_minimum_required(VERSION 3.1)

project(pitranger)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(libs)

add_executable(pitranger src/main.cpp src/imu.cpp src/motor_controller.cpp)
target_link_libraries(pitranger xsdevice fmt::fmt eigen roboteq libserial_static stdc++fs)
target_include_directories(pitranger PRIVATE src/${CMAKE_CURRENT_SOURCE_DIR})

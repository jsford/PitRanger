cmake_minimum_required(VERSION 3.1)

# Eigen3 Linear Algebra
add_library(eigen INTERFACE)
target_include_directories(eigen INTERFACE eigen3)

# Fmt for python-like string formatting
add_subdirectory(fmt)

# Motor Controller API
add_subdirectory(roboteq)

# XSens IMU API
add_subdirectory(xsdevice)

# Serial Port Library
add_subdirectory(libserial)

# Generate Python Bindings
add_subdirectory(pybind11)

# Command Line Argument Parsing
add_subdirectory(cxxopts)

# STB Image Processing Library
add_subdirectory(stb)

# Steam Controller Library
add_subdirectory(steam_controller)

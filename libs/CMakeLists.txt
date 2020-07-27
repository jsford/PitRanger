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
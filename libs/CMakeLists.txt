cmake_minimum_required(VERSION 3.15)

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

# Add Google RPC
include(FetchContent)
FetchContent_Declare(
  gRPC
  GIT_REPOSITORY https://github.com/grpc/grpc
  GIT_TAG        RELEASE_TAG_HERE  # e.g v1.28.0
)
set(FETCHCONTENT_QUIET OFF)
FetchContent_MakeAvailable(gRPC)

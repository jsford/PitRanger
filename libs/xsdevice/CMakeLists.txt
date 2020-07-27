cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 11)

# Build the xstypes library.
file(GLOB_RECURSE XSTYPES_SRCS xstypes/*.cpp xstypes/*.c xstypes/*.h)
add_library(xstypes ${XSTYPES_SRCS})
target_include_directories(xstypes PRIVATE xstypes/ PUBLIC include/xsens)

# Build the xcommunication library.
file(GLOB_RECURSE XCOMMUNICATION_SRCS xcommunication/*.cpp xcommunication/*.c xcommunication/*.h)
add_library(xcommunication ${XCOMMUNICATION_SRCS})
target_link_libraries(xcommunication xstypes)
target_include_directories(xcommunication PUBLIC xcommunication/ PUBLIC include/ include/xcommunication)

# Build the xsdevice library.
add_library(xsdevice xsdevice/deviceclass.cpp)
target_link_libraries(xsdevice PUBLIC xstypes xcommunication)
target_include_directories(xsdevice PUBLIC ./)

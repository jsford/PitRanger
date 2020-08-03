cmake_minimum_required(VERSION 3.1)

project(stb)

add_library(stb stb.cpp)
target_include_directories(stb PUBLIC ./)

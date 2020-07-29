cmake_minimum_required(VERSION 3.1)

add_executable(jog_ptu
    jog_ptu.cpp
)
target_link_libraries(jog_ptu
    libpitranger
)

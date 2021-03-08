cmake_minimum_required(VERSION 3.1)

add_executable(jog_ptu jog_ptu.cpp)
target_link_libraries(jog_ptu libpitranger)

add_executable(jog_wheels jog_wheels.cpp)
target_link_libraries(jog_wheels libpitranger)

add_executable(pitcap pitcap.cpp)
target_link_libraries(pitcap libpitranger)

add_executable(teleop teleop.cpp)
target_link_libraries(teleop libpitranger steam_controller)

set(CURSES_USE_NCURSES TRUE)
find_package(Curses REQUIRED)
add_executable(pitdriver pitdriver.cpp)

target_link_libraries(pitdriver libpitranger ${CURSES_LIBRARY})
target_include_directories(pitdriver PRIVATE ${CURSES_INCLUDE_DIR})

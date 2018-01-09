cmake_minimum_required(VERSION 2.8)

project(ros_ncurses)

find_package(roscpp REQUIRED)

set(CMAKE_CXX_STANDARD 11)

include_directories(
    include/
    ${roscpp_INCLUDE_DIRS}
)

add_executable(ros_ncurses
    src/main.cpp
    src/ncurses_node.cpp
)

target_link_libraries(ros_ncurses
    ncurses
    ${roscpp_LIBRARIES}
)

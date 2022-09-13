cmake_minimum_required(VERSION 3.0.2)

project(tello VERSION 0.1.1 LANGUAGES CXX)

add_library(tello INTERFACE)
add_library(tello::tello ALIAS tello)

target_include_directories(tello INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_sources(tello INTERFACE ${CMAKE_CURRENT_LIST_DIR}/tello.hpp)

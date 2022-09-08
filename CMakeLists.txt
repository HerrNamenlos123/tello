cmake_minimum_required(VERSION 3.16)

project(tello VERSION 0.1.1 LANGUAGES CXX)

add_library(tello INTERFACE)
add_library(tello::tello ALIAS tello)

target_compile_features(tello INTERFACE cxx_std_14)

target_include_directories(tello INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_sources(tello INTERFACE tello.hpp)

cmake_minimum_required(VERSION 3.0.2)

project(tello VERSION 0.1.1 LANGUAGES CXX)

add_library(tello INTERFACE)
add_library(tello::tello ALIAS tello)

target_include_directories(tello INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

# This only calls target_sources() if version is greater that 3.11.0. It's a 
# nice-to-have, but not necessary if compiling on an ancient system
set(version ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION})
if(${version} VERSION_GREATER_EQUAL 3.11.0)
target_sources(tello INTERFACE ${CMAKE_CURRENT_LIST_DIR}/tello.hpp)
endif()

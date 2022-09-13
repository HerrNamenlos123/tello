cmake_minimum_required(VERSION 3.0.2)
project(example LANGUAGES CXX)

# This example uses the library with the CMake INTERFACE library.
# Alternatively you can just copy and paste 'tello.hpp' into your include folder.
# In this case, omit the 'add_subdirectory' and 'target_link_libraries' below.

add_subdirectory(.. bin)    # Path to the library's CMakeLists.txt.

set(SOURCES
    src/main.cpp
)

add_executable(example ${SOURCES})
target_include_directories(example PUBLIC include)
target_link_libraries(example tello)                    # Depends on the tello INTERFACE library (consisting only of the header)

if (NOT MSVC)
    target_link_libraries(example pthread)    # pthread library must almost always be linked on linux
endif()

if (MSVC)
    target_compile_options(example PUBLIC /std:c++14)       # C++14 standard required
else()
    target_compile_options(example PUBLIC --std=c++14)      # C++14 standard required
endif()

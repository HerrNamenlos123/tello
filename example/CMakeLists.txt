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

target_compile_options()
set(CXX_STANDARD 14)                # C++14 standard required
set(CXX_STANDARD_REQUIRED ON)       # We need the standard
set(CXX_EXTENSIONS OFF)             # Strictly adhere to the standard, no compiler-specific additions

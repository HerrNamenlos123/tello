cmake_minimum_required(VERSION 3.0.2)
project(example LANGUAGES CXX)

add_subdirectory(.. bin)    # Path to the top-level CMakeLists.txt from the actual tello library. You can either find and link it like this
                            # or just copy-and-paste it into your 'include' directory (while omitting the target_link_libraries below)

set(SOURCES
    src/main.cpp
)

add_executable(example ${SOURCES})
target_include_directories(example PUBLIC include)
target_link_libraries(example tello)                    # Depends on the tello INTERFACE library (consisting only of the header)

set_target_properties(example PROPERTIES CXX_STANDARD 14)           # C++14 standard required
set_target_properties(example PROPERTIES CXX_STANDARD_REQUIRED ON)  # We need the standard
set_target_properties(example PROPERTIES CXX_EXTENSIONS OFF)        # Strictly adhere to the standard, no compiler-specific additions

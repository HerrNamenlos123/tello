cmake_minimum_required(VERSION 3.16)
project(example LANGUAGES CXX)

add_subdirectory(.. bin)    # Path to the top-level CMakeLists.txt from the actual tello library. You can either find and link it like this
                            # or just copy-and-paste it into your 'include' directory (while omitting the target_link_libraries below)

set(SOURCES
    src/main.cpp 
    src/tello.cpp
)

add_executable(example ${SOURCES})
target_include_directories(example PUBLIC include)
target_link_libraries(example tello)

if (MSVC)
    target_compile_definitions(example PRIVATE
       WIN32_LEAN_AND_MEAN      # Prevents Windows.h from adding unnecessary includes
       NOMINMAX                 # Prevents Windows.h from defining min/max as macros 
       _CRT_SECURE_NO_WARNINGS  # Prevents MSVC warnings about deprecated functions
    )
endif()

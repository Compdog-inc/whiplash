cmake_minimum_required(VERSION 3.20)
project(whiplash LANGUAGES CXX C)

# Force GCC compiler
if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    message(FATAL_ERROR "This project requires the GCC compiler. Detected: ${CMAKE_CXX_COMPILER_ID}")
endif()

# Set C++23 standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set C17 standard
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

include(FetchContent)

FetchContent_Declare(box2d
    GIT_REPOSITORY https://github.com/erincatto/box2d.git
    GIT_TAG v3.2.0
    GIT_SHALLOW 1
)

set(SDL_SHARED OFF CACHE BOOL "SDL_SHARED")
set(SDL_STATIC ON CACHE BOOL "SDL_STATIC")
FetchContent_MakeAvailable(box2d)

# Collect source files
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c" "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/**/*.c" "${CMAKE_CURRENT_SOURCE_DIR}/src/**/*.cpp")

# Define static library output (libwhiplash.a)
add_library(whiplash STATIC ${SOURCES})

# Specify include directory
target_include_directories(whiplash
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(whiplash PRIVATE
    box2d
)

# Compiler warnings and options
target_compile_options(whiplash
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

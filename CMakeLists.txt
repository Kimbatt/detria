cmake_minimum_required(VERSION 3.30)
cmake_policy(SET CMP0091 NEW)
cmake_policy(SET CMP0135 NEW)

if ("${CMAKE_BINARY_DIR}" STREQUAL "${CMAKE_SOURCE_DIR}") 
  message(FATAL_ERROR "In-source builds are disabled.
    Please create a subfolder and use `cmake ..` inside it.
    NOTE: cmake will now create CMakeCache.txt and CMakeFiles/*.
          You must delete them, or cmake will refuse to work.")
endif()

if (NOT EMSCRIPTEN)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

option (DETRIA_BUILD_BENCHMARKS "Build benchmarks" OFF)
option (DETRIA_BUILD_CXXMODULE "Build test project which uses C++20 modules" OFF)
option (DETRIA_COMPILE_WITH_CLANG_MSVC "Use Clang with MSVC" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

if (WIN32 AND DETRIA_COMPILE_WITH_CLANG_MSVC)
    set(CMAKE_GENERATOR_TOOLSET "ClangCL")
endif()

project(detria)

add_subdirectory(src)

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT detria-test)

if (DETRIA_BUILD_CXXMODULE)
    add_library(detria-module STATIC
        ${CMAKE_SOURCE_DIR}/include/detria.cppm
        ${CMAKE_SOURCE_DIR}/include/detria.hpp
    )

    target_sources(detria-module PUBLIC FILE_SET CXX_MODULES FILES
        ${CMAKE_SOURCE_DIR}/include/detria.cppm
    )

    set_target_properties(detria-module PROPERTIES CXX_STANDARD 20)
    set_target_properties(detria-module PROPERTIES CXX_SCAN_FOR_MODULES ON)
endif()

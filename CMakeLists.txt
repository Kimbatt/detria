cmake_minimum_required(VERSION 3.15)
cmake_policy(SET CMP0091 NEW)

if ("${CMAKE_BINARY_DIR}" STREQUAL "${CMAKE_SOURCE_DIR}") 
  message(FATAL_ERROR "In-source builds are disabled.
    Please create a subfolder and use `cmake ..` inside it.
    NOTE: cmake will now create CMakeCache.txt and CMakeFiles/*.
          You must delete them, or cmake will refuse to work.")
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

project (detria)

if (MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /permissive- /Wall /WX")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4459") # declaration of 'type' hides global declaration
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4820") # x bytes padding added after data member y
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd5045") # compiler will insert Spectre mitigation for memory load if /Qspectre switch specified
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4710") # function not inlined
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wall -Wextra -Wpedantic -Wcast-align -Wcast-qual -Wctor-dtor-privacy -Wdisabled-optimization -Wformat=2 -Winit-self -Wmissing-include-dirs -Wold-style-cast -Woverloaded-virtual -Wredundant-decls -Wshadow -Wsign-promo -Wundef -fdiagnostics-show-option -Wconversion")
    if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        # Ignored warnings:
        # -Wno-c++98-compat - the project requires C++17 or later
        # -Wno-padded - this is more like a diagnostic message, adding padding to structs is not a problem
        #               also, the struct layouts can change depending un used types
        # -Wno-float-equal - exact floating point comparisons are required for robust predicates
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wconditional-uninitialized -Weverything -Wmost -Wno-c++98-compat -Wno-padded -Wno-float-equal")
    endif()
endif ()

add_subdirectory(src)

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT detria-test)

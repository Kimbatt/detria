if ((CMAKE_CXX_COMPILER_ID STREQUAL "Clang") OR (CMAKE_GENERATOR_TOOLSET STREQUAL ClangCL))
    set(CLANG TRUE)
endif()

if (MSVC AND NOT CLANG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /permissive- /Wall /WX")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4459") # declaration of 'type' hides global declaration
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4820") # x bytes padding added after data member y
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd5045") # compiler will insert Spectre mitigation for memory load if /Qspectre switch specified
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4710") # function not inlined
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4711") # function selected for automatic inline expansion
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wall -Wextra -Wpedantic -Wcast-align -Wcast-qual -Wctor-dtor-privacy -Wdisabled-optimization -Wformat=2 -Winit-self -Wmissing-include-dirs -Wold-style-cast -Woverloaded-virtual -Wredundant-decls -Wshadow -Wsign-promo -Wundef -Wconversion")
    if (CLANG)
        # Ignored warnings:
        # -Wno-c++98-compat, -Wno-c++98-compat-pedantic - the project requires C++17 or later
        # -Wno-padded - this is more like a diagnostic message, adding padding to structs is not a problem
        #               also, the struct layouts can change depending un used types
        # -Wno-float-equal - exact floating point comparisons are required for robust predicates
        # -Wno-unsafe-buffer-usage - pointers are used in the project
        # -Wno-switch-default - this warns if all enumeration values are handled, but the switch has no default
        # Also, -Wno-error=unknown-warning-option is used, because -Wno-unsafe-buffer-usage doesn't exist in older Clang versions
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=unknown-warning-option -Wconditional-uninitialized -Weverything -Wmost -Wno-c++98-compat -Wno-padded -Wno-float-equal -Wno-c++98-compat-pedantic -Wno-unsafe-buffer-usage -Wno-switch-default")
    endif()
    if (EMSCRIPTEN)
        # Ignored warnings for emscripten:
        # -Wno-unused-template - file reading related functions are not used
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-template")
    endif()
endif ()

if (EMSCRIPTEN)
    set(CMAKE_EXECUTABLE_SUFFIX ".html")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -s ALLOW_MEMORY_GROWTH -s TOTAL_MEMORY=128MB")
endif()

add_executable(detria-test main.cpp ${CMAKE_SOURCE_DIR}/include/detria.hpp)

target_include_directories(detria-test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(detria-test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../common)
target_include_directories(detria-test PRIVATE ${CMAKE_SOURCE_DIR})
target_include_directories(detria-test PRIVATE ${CMAKE_SOURCE_DIR}/include)

set_target_properties(detria-test PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")

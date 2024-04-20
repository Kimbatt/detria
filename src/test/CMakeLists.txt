if (MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /permissive- /Wall /WX")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4459") # declaration of 'type' hides global declaration
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4820") # x bytes padding added after data member y
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd5045") # compiler will insert Spectre mitigation for memory load if /Qspectre switch specified
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4710") # function not inlined
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4711") # function selected for automatic inline expansion
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

if (EMSCRIPTEN)
    set(CMAKE_EXECUTABLE_SUFFIX ".html")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -s ALLOW_MEMORY_GROWTH -s TOTAL_MEMORY=128MB")
endif()

file(GLOB_RECURSE FILES_SRC *.cpp *.h *.hpp *.inl)

add_executable(detria-test ${FILES_SRC})

target_include_directories(detria-test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(detria-test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../common)
target_include_directories(detria-test PRIVATE ${CMAKE_SOURCE_DIR})
target_include_directories(detria-test PRIVATE ${CMAKE_SOURCE_DIR}/include)

set_target_properties(detria-test PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")

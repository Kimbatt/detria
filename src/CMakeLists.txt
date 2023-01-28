
if (EMSCRIPTEN)
    set(CMAKE_EXECUTABLE_SUFFIX ".html")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -s ALLOW_MEMORY_GROWTH -s TOTAL_MEMORY=128MB")
endif()

file(GLOB_RECURSE FILES_SRC *.cpp *.h *.hpp *.inl)

add_executable(detria-test ${FILES_SRC})

target_include_directories(detria-test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(detria-test PRIVATE ${CMAKE_SOURCE_DIR})

set_target_properties(detria-test PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")

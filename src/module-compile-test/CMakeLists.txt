add_executable(detria-module-test
    main.cpp
)

if (EMSCRIPTEN)
    set(CMAKE_EXECUTABLE_SUFFIX ".html")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -s ALLOW_MEMORY_GROWTH -s TOTAL_MEMORY=128MB")
endif()

target_link_libraries(detria-module-test detria-module)

target_include_directories(detria-module-test PRIVATE ${CMAKE_SOURCE_DIR})
target_include_directories(detria-module-test PRIVATE ${CMAKE_SOURCE_DIR}/include)
set_target_properties(detria-module-test PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
set_target_properties(detria-module-test PROPERTIES CXX_STANDARD 20)
set_target_properties(detria-module-test PROPERTIES CXX_STANDARD_REQUIRED ON)
set_target_properties(detria-module-test PROPERTIES CXX_SCAN_FOR_MODULES ON)

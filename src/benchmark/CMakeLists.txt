if (EMSCRIPTEN)
    set(CMAKE_EXECUTABLE_SUFFIX ".html")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -s ALLOW_MEMORY_GROWTH -s TOTAL_MEMORY=128MB")
endif()

include(FetchContent)

# Fetch other triangulation libraries

FetchContent_Declare(
    poly2tri
    GIT_REPOSITORY      https://github.com/jhasse/poly2tri
    GIT_TAG             13d64e75a84dd8f36673a7eaf022e8e9fdd1f88b
)

FetchContent_Declare(
    cdt
    GIT_REPOSITORY      https://github.com/artem-ogre/CDT
    GIT_TAG             58f34da24b438bd17629450fcec189ccb181dc9f
)

FetchContent_MakeAvailable(poly2tri cdt)

add_executable(detria-benchmark
    main.cpp
    boxplot.cpp
    boxplot.h
    triangulator_benchmark.h
    triangulator_benchmark.cpp
    impl/detria.cpp
    impl/triangle.cpp
    impl/poly2tri.cpp
    impl/cdt.cpp
)

target_include_directories(detria-benchmark PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(detria-benchmark PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../common)
target_include_directories(detria-benchmark PRIVATE ${CMAKE_SOURCE_DIR})
target_include_directories(detria-benchmark PRIVATE ${CMAKE_SOURCE_DIR}/include)
set_target_properties(detria-benchmark PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
set_target_properties(detria-benchmark PROPERTIES CXX_STANDARD 20)
set_target_properties(detria-benchmark PROPERTIES CXX_STANDARD_REQUIRED ON)

# Include fetched libraries
target_include_directories(detria-benchmark PRIVATE ${nanobench_SOURCE_DIR}/src/include)
target_include_directories(detria-benchmark PRIVATE ${poly2tri_SOURCE_DIR}/poly2tri)
target_include_directories(detria-benchmark PRIVATE ${cdt_SOURCE_DIR}/CDT/include)

set_target_properties(poly2tri PROPERTIES FOLDER Benchmark)

target_link_libraries(detria-benchmark poly2tri)

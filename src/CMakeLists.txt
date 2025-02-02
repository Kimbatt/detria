add_subdirectory(test)

if (DETRIA_BUILD_CXXMODULE)
    add_subdirectory(module-compile-test)
endif()

if (DETRIA_BUILD_BENCHMARKS)
    add_subdirectory(benchmark)
endif()

add_subdirectory(map_metrics)

if (BUILD_PYTHON_MODULE)
    # TODO (achains): Remove add_subdir
    add_subdirectory("${CMAKE_SOURCE_DIR}/external/pybind11" "${CMAKE_BINARY_DIR}/external/pybind11")
    add_subdirectory(pybind)
endif()

if (BUILD_TESTS)
    add_subdirectory(tests)
endif()

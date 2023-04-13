enable_testing()

add_subdirectory(gtest)

# Copying data from inputData to binary dir
# TODO (achains): Change this
file(GLOB PC_DATA ${CMAKE_SOURCE_DIR}/cpp/tests/data)
file(COPY ${PC_DATA} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

add_executable(unit-tests
    main.cpp
    parse_utils.cpp
    pc_utils.cpp
    test_depth.cpp
    test_orthogonal.cpp
    test_parse.cpp
    test_base_metrics.cpp
)

#find_package(Eigen3 3.4 REQUIRED)
#if (Eigen3_FOUND)
#    target_link_libraries(unit-tests PRIVATE Eigen3::Eigen)
#endif()

target_link_libraries(unit-tests PRIVATE ${cilantro_LIBRARIES})
target_include_directories(unit-tests PRIVATE "${CMAKE_BINARY_DIR}/external/cilantro/cilantro/include")

target_link_libraries(unit-tests PRIVATE gtest_main)
target_link_libraries(unit-tests PRIVATE map_metrics)

target_compile_definitions(unit-tests PRIVATE
    TEST_ORTHOGONAL_METRICS 
    TEST_BASE_DEPTH_METRICS
)

include(GoogleTest)
gtest_discover_tests(unit-tests)
pybind11_add_module(pybind metrics.cpp)

find_package(Eigen3 3.4 REQUIRED)

# TODO: remove?
target_link_libraries(pybind PRIVATE ${cilantro_LIBRARIES})
target_include_directories(pybind PRIVATE "${CMAKE_BINARY_DIR}/external/cilantro/cilantro/include")

target_link_libraries(pybind PRIVATE Eigen3::Eigen)
target_link_libraries(pybind PUBLIC map_metrics)

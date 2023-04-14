pybind11_add_module(pybind
        map_metrics_pybind.cpp
        config.cpp
        map_tree.cpp
        metrics.cpp
        utils/cloud_utils.cpp)

find_package(Eigen3 3.4 REQUIRED)

target_link_libraries(pybind PRIVATE Eigen3::Eigen)
target_link_libraries(pybind PUBLIC map_metrics)

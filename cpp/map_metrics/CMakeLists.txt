set(TARGET_NAME map_metrics_tmp)

set(TARGET_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(TARGET_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(SRC_FILES
    ${TARGET_SOURCE_DIR}/map_metrics/config.cpp
    ${TARGET_SOURCE_DIR}/map_metrics/metric_estimator.cpp
    ${TARGET_SOURCE_DIR}/map_metrics/point_statistics.cpp
    ${TARGET_SOURCE_DIR}/map_metrics/utils/kdtree_utils.cpp
    ${TARGET_SOURCE_DIR}/map_metrics/utils/map_utils.cpp)

add_library(${TARGET_NAME} ${SRC_FILES})

find_package(Eigen3 REQUIRED)
find_package(cilantro REQUIRED)

target_include_directories(
    ${TARGET_NAME}
    SYSTEM
    PUBLIC
    $<BUILD_INTERFACE:${TARGET_INCLUDE_DIR}>
)
# TODO (achains): Remove this
target_include_directories(${TARGET_NAME} PRIVATE "${CMAKE_BINARY_DIR}/external/cilantro/cilantro/include")
target_link_libraries(${TARGET_NAME} PUBLIC Eigen3::Eigen)
target_link_libraries(${TARGET_NAME} PRIVATE ${cilantro_LIBRARIES})
#####################################
# Define Target Information
#####################################
set(TARGET_NAME map_metrics)

set(TARGET_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(TARGET_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(SRC_FILES
    ${TARGET_SOURCE_DIR}/map_metrics/metric_estimator.cpp
    ${TARGET_SOURCE_DIR}/map_metrics/point_statistics.cpp
    ${TARGET_SOURCE_DIR}/map_metrics/utils/kdtree_utils.cpp
    ${TARGET_SOURCE_DIR}/map_metrics/utils/map_utils.cpp
)

#####################################
# Required packages
#####################################
find_package(Eigen3 REQUIRED)
find_package(cilantro REQUIRED)

#####################################
# Define Target
#####################################
add_library(${TARGET_NAME} ${SRC_FILES})

#####################################
# Define Target Properties
#####################################
set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 14)
target_compile_features(${TARGET_NAME} PUBLIC cxx_std_14)

target_include_directories(
    ${TARGET_NAME}
    SYSTEM
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(${TARGET_NAME} PUBLIC Eigen3::Eigen)
target_link_libraries(${TARGET_NAME} PRIVATE cilantro)

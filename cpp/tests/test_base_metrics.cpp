//
// Created by achains on 30.10.2021.
//

#ifdef TEST_BASE_LIDAR_METRICS

#include <gtest/gtest.h>
#include <chrono>

#include "parse_utils.h"

#include "metrics.h"

std::vector<std::vector<double>> GetCalibrationMatrix(){
    return
            {
                    {4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03, -1.198459927713e-02},
                    {-7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01, -5.403984729748e-02},
                    {9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03, -2.921968648686e-01},
                    {0, 0, 0, 1}
            };
}

std::vector<Eigen::Matrix4d> GetCalibratedTrajectory(std::filesystem::path const & path){
    std::vector<std::vector<double>> trajectory = parse_utils::ParseTrajectory("data/00.txt");

    return parse_utils::CalibrateTrajectory(GetCalibrationMatrix(), trajectory);
}

std::vector<cilantro::VectorSet3d> GetPointClouds(std::filesystem::path const & path){
    std::vector<std::vector<double>> PCs_vector = parse_utils::GetPointClouds(path);
    std::vector<cilantro::VectorSet3d> PCs_points;

    for (auto & PC_vector : PCs_vector)
    {
        cilantro::VectorSet3d points = parse_utils::VectorToPointCloudPoints(PC_vector);
        PCs_points.push_back(points);
    }

    return PCs_points;
}

TEST(LidarTest, MPV){
    std::vector<Eigen::Matrix4d> tj_gt = GetCalibratedTrajectory("data/00.txt");
    ASSERT_EQ(tj_gt.size(), 20);

    std::vector<cilantro::VectorSet3d> PCs = GetPointClouds("data/kitti_00");

    auto start_time = std::chrono::system_clock::now();
    double result_mpv = metrics::GetMPV(std::vector<cilantro::VectorSet3d>(PCs.begin(), PCs.begin() + 5),
                                        std::vector<Eigen::Matrix4d>(tj_gt.begin(), tj_gt.begin() + 5));
    auto elapsed_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now() - start_time);

    std::cout << "MPV: " << result_mpv << std::endl;
    std::cout << "Elapsed time (ms): " << elapsed_milliseconds.count() << std::endl;

    ASSERT_LE(std::fabs(result_mpv - 0.019802606170394272), 1e-8);
}

TEST(LidarTest, MME){
    std::vector<Eigen::Matrix4d> tj_gt = GetCalibratedTrajectory("data/00.txt");
    ASSERT_EQ(tj_gt.size(), 20);

    std::vector<cilantro::VectorSet3d> PCs = GetPointClouds("data/kitti_00");

    auto start_time = std::chrono::system_clock::now();
    double result_mme = metrics::GetMME(std::vector<cilantro::VectorSet3d>(PCs.begin(), PCs.begin() + 5),
                                        std::vector<Eigen::Matrix4d>(tj_gt.begin(), tj_gt.begin() + 5));
    auto elapsed_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now() - start_time);

    std::cout << "MME: " << result_mme << std::endl;
    std::cout << "Elapsed time (ms): " << elapsed_milliseconds.count() << std::endl;

    ASSERT_LE(std::fabs(result_mme - 0.018431360563601673), 1e-8);
}

#endif // TEST_BASE_LIDAR_METRICS

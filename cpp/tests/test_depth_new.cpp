// Copyright (c) 2022, Arthur Saliou, Anastasiia Kornilova
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//
//
//  Created on: May 20, 2022
//       Author: Arthur Saliou
//               arthur.salio@gmail.com
//
#include <gtest/gtest.h>
#include <chrono>
#include <iostream>

#include "parse_utils.h"

#include "map_metrics/metric_estimator.h"
#include "map_metrics/utils/cloud_utils.h"

TEST(DepthTest, MME){
  std::vector<cilantro::VectorSet3d> pcs = parse_utils::depth_parse::GetDepthPCs("data/depth/pcs");
  std::vector<Eigen::Matrix4d> poses = parse_utils::depth_parse::GetDepthPoses("data/depth/poses");
  ASSERT_EQ(pcs.size(), 28);    ASSERT_EQ(pcs.size(), poses.size());

  auto start_time = std::chrono::system_clock::now();
  auto points_map = map_metrics::aggregateMap(pcs, poses);
  auto estimator = map_metrics::MetricsEstimator(points_map, map_metrics::Config(5, 0.2, 30, 5));
  double actual_mme = estimator.MME();
  auto elapsed_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now() - start_time);

  std::cout << "MME: " << actual_mme << std::endl;
  std::cout << "Elapsed time (ms): " << elapsed_milliseconds.count() << std::endl;

  ASSERT_LE(std::fabs(actual_mme - -3.6144387057257523), 1e-8);
}

//TEST(DepthTest, MPV){
//  std::vector<cilantro::VectorSet3d> pcs = parse_utils::depth_parse::GetDepthPCs("data/depth/pcs");
//  std::vector<Eigen::Matrix4d> poses = parse_utils::depth_parse::GetDepthPoses("data/depth/poses");
//  ASSERT_EQ(pcs.size(), 28);    ASSERT_EQ(pcs.size(), poses.size());
//
//  auto start_time = std::chrono::system_clock::now();
//  double actual_mpv = metrics::GetMPV(pcs, poses, config::CustomConfig(5, 0.2, 30, 5));
//  auto elapsed_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
//      std::chrono::system_clock::now() - start_time);
//
//  std::cout << "MPV: " << actual_mpv << std::endl;
//  std::cout << "Elapsed time (ms): " << elapsed_milliseconds.count() << std::endl;
//
//  ASSERT_LE(std::fabs(actual_mpv - 0.0025390347963358565), 1e-8);
//}
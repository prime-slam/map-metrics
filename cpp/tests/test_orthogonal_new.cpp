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

#include "parse_utils.h"

#include "map_metrics/metric_estimator.h"
#include "map_metrics/utils/cloud_utils.h"

#include <chrono>
#include <iostream>

TEST(Orthogonal, MOM) {
  std::vector<cilantro::VectorSet3d> PCs = parse_utils::depth_parse::GetDepthPCs("data/depth/pcs");
  std::vector<Eigen::Matrix4d> poses = parse_utils::depth_parse::GetDepthPoses("data/depth/poses");
  auto pcd = PCs[2];  // PC-0091.ply

  auto orth_subset = map_metrics::findOrthogonalSubset(pcd, map_metrics::Config(5, 0.2, 30, 5));

  auto start_time = std::chrono::system_clock::now();
  auto points_map = map_metrics::aggregateMap(PCs, poses);
  auto estimator = map_metrics::MetricsEstimator(points_map, map_metrics::Config(5, 0.2, 30, 5));

  double result_mom = estimator.MOM(orth_subset);

  auto elapsed_milliseconds =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time);

  std::cout << "MOM: " << result_mom << std::endl;
  std::cout << "Elapsed time (ms): " << elapsed_milliseconds.count() << std::endl;

  ASSERT_LE(std::fabs(result_mom - 0.00203637), 1e-8);
}
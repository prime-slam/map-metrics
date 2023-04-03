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
#ifndef MAP_METRICS_ORTHOGONAL_H
#define MAP_METRICS_ORTHOGONAL_H
#include <vector>

#include <Eigen/Core>

#include "config.h"

namespace map_metrics {
std::vector<Eigen::Matrix3Xd> findOrthogonalSubset(Eigen::Matrix3Xd const& points, Config const& config);

std::vector<Eigen::Index> findPlanarRegions(Eigen::Matrix3Xd const& points, double knn_rad);
}  // namespace map_metrics
#endif  // MAP_METRICS_ORTHOGONAL_H

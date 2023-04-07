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
#ifndef MAP_METRICS_KDTREE_UTILS_H
#define MAP_METRICS_KDTREE_UTILS_H
#include <vector>

#include <cilantro/core/kd_tree.hpp>

namespace map_metrics {
std::vector<Eigen::Index> getRadiusSearchIndices(cilantro::KDTree3d<> const& tree, Eigen::Vector3d const& query,
                                                 double radius);

std::vector<Eigen::Index> findPlanarPoints(cilantro::KDTree3d<> const& tree, double radius);
}  // namespace map_metrics
#endif  // MAP_METRICS_KDTREE_UTILS_H

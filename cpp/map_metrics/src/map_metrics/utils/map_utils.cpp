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
#include "map_metrics/utils/map_utils.h"

#include <cilantro/utilities/point_cloud.hpp>

namespace map_metrics {
Eigen::Matrix3Xd aggregateMap(std::vector<Eigen::Matrix3Xd> const& point_sequence,
                              std::vector<Eigen::Matrix4d> const& poses) {
  if (point_sequence.size() != poses.size()) {
    throw std::runtime_error("Point sequence size != Poses size (" + std::to_string(point_sequence.size()) +
                             " != " + std::to_string(poses.size()) + ")");
  }

  cilantro::PointCloud3d map{};
  Eigen::Matrix4d center = poses[0].inverse();

  for (size_t i = 0; i < point_sequence.size(); ++i) {
    cilantro::RigidTransform3d transform_mx(center * poses[i]);
    map.append(cilantro::PointCloud3d(point_sequence[i]).transformed(transform_mx));
  }

  return map.points;
}
}  // namespace map_metrics
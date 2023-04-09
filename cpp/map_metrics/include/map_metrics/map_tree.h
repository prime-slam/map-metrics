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
#ifndef MAP_METRICS_MAP_TREE_H
#define MAP_METRICS_MAP_TREE_H
#include <memory>

#include <Eigen/Core>

#include "map_metrics/config.h"

namespace map_metrics {
class MapTree {
 public:
  MapTree(Eigen::Matrix3Xd const& points, double knn_rad);

  ~MapTree();

  MapTree(MapTree&& op) noexcept;

  MapTree& operator=(MapTree&& op) noexcept;

  std::vector<std::vector<Eigen::Index>> const& getMapNeighbours() const;

  std::vector<std::vector<Eigen::Index>> getNeighboursByComponent(Eigen::Matrix3Xd const& point_component) const;

  Eigen::Matrix3Xd getMapPoints(std::vector<Eigen::Index> const& point_indices) const;

 private:
  class MapTreeImpl;
  std::unique_ptr<MapTreeImpl> impl_;
};
}  // namespace map_metrics
#endif  // MAP_METRICS_MAP_TREE_H

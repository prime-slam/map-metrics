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
#include "kdtree_utils.h"

namespace map_metrics {
Eigen::MatrixX3d transformPointIdxToMatrix(Eigen::Ref<const Eigen::Matrix3Xd> const& points,
                                           std::vector<Eigen::Index> const& idx_vector) {
  Eigen::MatrixX3d matrix(idx_vector.size(), 3);
  Eigen::Index row_idx = 0;
  for (Eigen::Index i : idx_vector) {
    matrix.row(row_idx++) = points.col(i);
  }
  return matrix;
}

std::vector<Eigen::Index> getRadiusSearchIndices(cilantro::KDTree3d<> const& tree,
                                                 Eigen::Ref<const Eigen::Vector3d> const& query, double radius) {
  cilantro::NeighborSet<double> nn = tree.radiusSearch(query, pow(radius, 2.0));
  std::vector<Eigen::Index> idx_vector(nn.size());
  std::transform(nn.begin(), nn.end(), idx_vector.begin(), [](auto n) { return n.index; });
  return idx_vector;
}
}  // namespace map_metrics
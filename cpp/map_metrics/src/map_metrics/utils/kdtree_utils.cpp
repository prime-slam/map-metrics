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

#include "map_metrics/utils/cloud_utils.h"

namespace map_metrics {
std::vector<Eigen::Index> findPlanarPoints(cilantro::KDTree3d<> const& tree, double radius) {
  std::vector<Eigen::Index> normals_indices;
  for (Eigen::Index i = 0; i < tree.getPointsMatrixMap().cols(); ++i) {
    auto neighbours_idx = getRadiusSearchIndices(tree, tree.getPointsMatrixMap().col(i), radius);

    int32_t component_inner_min_knn = 3;
    bool enough_neighbours = neighbours_idx.size() > component_inner_min_knn;
    if (enough_neighbours) {
      Eigen::Matrix3d cov_matrix = findCovariance(tree.getPointsMatrixMap()(Eigen::all, neighbours_idx));
      Eigen::VectorXd eigenvalues = cov_matrix.eigenvalues().real();
      std::sort(eigenvalues.begin(), eigenvalues.end());
      // TODO (achains): Better planarity check?
      if (100 * eigenvalues[0] < eigenvalues[1]) {
        normals_indices.push_back(i);
      }
    }
  }

  return normals_indices;
}
}  // namespace map_metrics
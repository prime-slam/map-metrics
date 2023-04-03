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
#include "map_metrics/orthogonal.h"

#include <memory>

#include <cilantro/utilities/point_cloud.hpp>

#include "clustering.h"
#include "max_clique.h"
#include "point_statistics.h"
#include "utils/kdtree_utils.h"

namespace map_metrics {
std::vector<Eigen::Matrix3Xd> findOrthogonalSubset(Eigen::Matrix3Xd const& points, Config const& config) {
  auto planar_indices = findPlanarRegions(points, config.knn_rad);
  auto pcd = cilantro::PointCloud3d(points);
  // TODO (achains): pow(knn_rad, 2)?
  pcd.estimateNormalsKNNInRadius(config.max_nn, config.knn_rad, true);

  Eigen::Matrix3Xd planar_points = pcd.points(Eigen::all, planar_indices);
  Eigen::Matrix3Xd planar_normals = pcd.normals(Eigen::all, planar_indices);

  // TODO (achains): Move to config
  double distance_threshold = 1e-1;
  auto clusterizer = clusterizeAHC(planar_normals, distance_threshold);
  clusterizer.filterClusters(planar_normals, config.min_clust_size);

  std::vector<Eigen::Index> max_clique = findMaxClique(clusterizer, distance_threshold);

  std::vector<Eigen::Matrix3Xd> orthogonal_subset;
  orthogonal_subset.reserve(max_clique.size());

  for (Eigen::Index idx : max_clique) {
    auto idx_mask = (clusterizer.getLabels().array() == clusterizer.getIdx()[idx]);
    Eigen::Matrix3Xd new_points(3, idx_mask.count());
    Eigen::Index new_points_idx = 0;
    for (Eigen::Index i = 0; i < idx_mask.size(); ++i) {
      if (idx_mask[i]) new_points.col(new_points_idx++) = planar_points.col(i);
    }
    orthogonal_subset.push_back(new_points);
  }

  return orthogonal_subset;
}

std::vector<Eigen::Index> findPlanarRegions(Eigen::Matrix3Xd const& points, double knn_rad) {
  auto kd_tree = std::make_unique<cilantro::KDTree3d<>>(points);

  std::vector<Eigen::Index> normals_indices;
  for (Eigen::Index i = 0; i < kd_tree->getPointsMatrixMap().cols(); ++i) {
    auto neighbours_idx = getRadiusSearchIndices(*kd_tree, kd_tree->getPointsMatrixMap().col(i), knn_rad);

    int32_t component_inner_min_knn = 3;
    bool enough_neighbours = neighbours_idx.size() > component_inner_min_knn;
    if (enough_neighbours) {
      Eigen::MatrixX3d cov_matrix = findCovariance(kd_tree->getPointsMatrixMap()(Eigen::all, neighbours_idx));
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
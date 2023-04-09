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
#include "map_metrics/utils/cloud_utils.h"

#include <cilantro/utilities/point_cloud.hpp>

#include "cluster_means.h"
#include "kdtree_utils.h"
#include "max_clique_visitor.h"

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

std::vector<Eigen::Matrix3Xd> findOrthogonalSubset(Eigen::Matrix3Xd const& points, Config const& config) {
  auto planar_indices = findPlanarPoints(MapTree(points, config.knn_rad));
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

Eigen::Matrix3d findCovariance(Eigen::Matrix3Xd const& points) {
  Eigen::Matrix3Xd centered = points.colwise() - points.rowwise().mean();
  return (centered * centered.adjoint()) / (static_cast<double>(points.cols()) - 1.0);
}
}  // namespace map_metrics
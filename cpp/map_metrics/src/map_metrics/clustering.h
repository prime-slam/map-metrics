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
#ifndef MAP_METRICS_CLUSTERING_H
#define MAP_METRICS_CLUSTERING_H

#include <alglib/ap.h>
#include <cstdint>
#include <vector>

#include <Eigen/Core>

namespace map_metrics {
class ClusterMeans {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit ClusterMeans(alglib::integer_1d_array const& labels, Eigen::Index cluster_number);

  void filterClusters(Eigen::Ref<const Eigen::Matrix3Xd> const points, int32_t min_clust_size);

  const Eigen::Matrix3Xd& getMeans() const;

  const Eigen::VectorXi& getIdx() const;

  const Eigen::VectorXi& getLabels() const;

  Eigen::Index getClusterNumber() const;

 private:
  Eigen::VectorXi labels_;
  Eigen::Matrix3Xd cluster_means_;
  Eigen::VectorXi cluster_idx_;
  Eigen::Index cluster_number_;
};

ClusterMeans clusterizeAHC(Eigen::Ref<const Eigen::Matrix3Xd> const points, double distance_treshold);

}  // namespace map_metrics

#endif  // MAP_METRICS_CLUSTERING_H
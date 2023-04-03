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
#ifndef MAP_METRICS_MAX_CLIQUE_H
#define MAP_METRICS_MAX_CLIQUE_H

#include <memory>
#include <vector>

#include <iostream>
#include "boost/graph/adjacency_matrix.hpp"
#include "clustering.h"

namespace map_metrics {
class MaxCliqueVisitor {
 public:
  MaxCliqueVisitor(ClusterMeans const& clusterizer, std::shared_ptr<Eigen::Index> max_covered_points,
                   std::shared_ptr<std::vector<Eigen::Index>> max_clique_idx);

  template <typename Clique, typename Graph>
  void clique(Clique const& c, Graph const& g) {
    if (c.size() == 2) return;

    std::vector<Eigen::Index> clique_idx;
    clique_idx.reserve(c.size());

    Eigen::Index number_of_points = 0;

    typename Clique::const_iterator i, end = c.end();
    for (i = c.begin(); i != end; ++i) {
      number_of_points += (clusterizer_.getLabels().array() == clusterizer_.getIdx()[*i]).count();
      clique_idx.push_back(*i);
    }

    if (number_of_points > *max_covered_points_) {
      *max_covered_points_ = number_of_points;
      *max_clique_idx_ = clique_idx;
    }
  }

 private:
  ClusterMeans clusterizer_;
  std::shared_ptr<Eigen::Index> max_covered_points_;
  std::shared_ptr<std::vector<Eigen::Index>> max_clique_idx_;
};

std::vector<Eigen::Index> findMaxClique(ClusterMeans const& clusterizer, double eps);
}  // namespace map_metrics

#endif  // MAP_METRICS_MAX_CLIQUE_H

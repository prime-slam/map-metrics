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
#include "metrics_core.h"

#include <algorithm>
#include <numeric>

namespace map_metrics {
double baseMetricEstimator(MapTree const& map_tree, double (*point_statistic)(Eigen::Matrix3Xd const& points),
                           int min_component_size) {
  std::vector<double> metric_statistic;
  for (auto const& neighbour_list : map_tree.getMapNeighbours()) {
    bool enough_neighbours = neighbour_list.size() > min_component_size;
    if (enough_neighbours) {
      metric_statistic.push_back(point_statistic(map_tree.getMapPoints(neighbour_list)));
    }
  }

  return std::accumulate(metric_statistic.begin(), metric_statistic.end(), 0.0) /
         static_cast<double>(metric_statistic.size());
}

double orthogonalEstimator(MapTree const& map_tree, double (*point_statistic)(Eigen::Matrix3Xd const& points),
                           int min_component_size, std::vector<Eigen::Matrix3Xd> const& orthogonal_subset) {
  std::vector<double> metric_statistic;
  for (auto const& orthogonal_component : orthogonal_subset) {
    std::vector<double> component_metric_statistic;

    for (auto const& neighbour_list : map_tree.getNeighboursByComponent(orthogonal_component)) {
      bool enough_neighbours = neighbour_list.size() > min_component_size;
      if (enough_neighbours) {
        component_metric_statistic.push_back(point_statistic(map_tree.getMapPoints(neighbour_list)));
      }
    }
    if (component_metric_statistic.empty()) continue;
    std::sort(component_metric_statistic.begin(), component_metric_statistic.end());
    double component_statistic_median = (component_metric_statistic.size() % 2 == 0
                                             ? (component_metric_statistic[component_metric_statistic.size() / 2 - 1] +
                                                component_metric_statistic[component_metric_statistic.size() / 2]) /
                                                   2.0
                                             : component_metric_statistic[component_metric_statistic.size() / 2]);
    metric_statistic.push_back(component_statistic_median);
  }

  return std::accumulate(metric_statistic.begin(), metric_statistic.end(), 0.0);
}
}  // namespace map_metrics
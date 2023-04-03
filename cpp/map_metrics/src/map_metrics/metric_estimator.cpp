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
#include "map_metrics/metric_estimator.h"
#include "point_statistics.h"
#include "utils/kdtree_utils.h"

#include <algorithm>
#include <numeric>

namespace map_metrics {
class MetricsEstimator::Impl {
 public:
  Impl(Eigen::Matrix3Xd const& points, Config const& config);

  double MME();

  double MPV();

  double MOM(std::vector<Eigen::Matrix3Xd> const& orthogonal_subset = {});

 private:
  std::unique_ptr<cilantro::KDTree3d<>> map_kd_tree_;
  Config config_;

  double baseMetricEstimator(double (*metric)(Eigen::Matrix3Xd const& points));

  double orthogonalEstimator(double (*metric)(Eigen::Matrix3Xd const& points),
                             std::vector<Eigen::Matrix3Xd> const& orthogonal_subset);
};

MetricsEstimator::MetricsEstimator(Eigen::Matrix3Xd const& points, map_metrics::Config const& config)
    : impl_(new Impl(points, config)) {}

MetricsEstimator::~MetricsEstimator() = default;

MetricsEstimator::MetricsEstimator(map_metrics::MetricsEstimator&& op) noexcept = default;

MetricsEstimator& MetricsEstimator::operator=(map_metrics::MetricsEstimator&& op) noexcept = default;

double MetricsEstimator::MME() { return impl_->MME(); }

double MetricsEstimator::MPV() { return impl_->MPV(); }

double MetricsEstimator::MOM() { return impl_->MOM(); }

MetricsEstimator::Impl::Impl(Eigen::Matrix3Xd const& points, map_metrics::Config const& config)
    : map_kd_tree_(std::make_unique<cilantro::KDTree3d<>>(points)), config_(config) {}

double MetricsEstimator::Impl::MME() { return baseMetricEstimator(computePointsEntropy); }

double MetricsEstimator::Impl::MPV() { return baseMetricEstimator(computePointsVariance); }

double MetricsEstimator::Impl::MOM(std::vector<Eigen::Matrix3Xd> const& orthogonal_subset) {
  if (orthogonal_subset.empty()) {
    ;
  }
  return orthogonalEstimator(computePointsVariance, orthogonal_subset);
}

double MetricsEstimator::Impl::baseMetricEstimator(double (*metric)(Eigen::Matrix3Xd const& points)) {
  std::vector<double> metric_statistic;
  for (auto const& point : map_kd_tree_->getPointsMatrixMap().colwise()) {
    auto neighbours_idx = getRadiusSearchIndices(*map_kd_tree_, point, config_.knn_rad);

    bool enough_neighbours = neighbours_idx.size() > config_.min_knn;
    if (enough_neighbours) {
      metric_statistic.push_back(metric(map_kd_tree_->getPointsMatrixMap()(Eigen::all, neighbours_idx)));
    }
  }

  return std::accumulate(metric_statistic.begin(), metric_statistic.end(), 0.0) /
         static_cast<double>(metric_statistic.size());
}

double MetricsEstimator::Impl::orthogonalEstimator(double (*metric)(Eigen::Matrix3Xd const& points),
                                                   std::vector<Eigen::Matrix3Xd> const& orthogonal_subset) {
  std::vector<double> metric_statistic;
  for (auto const& orthogonal_component : orthogonal_subset) {
    std::vector<double> component_metric_statistic;
    for (auto const& point : orthogonal_component.colwise()) {
      auto neighbours_idx = getRadiusSearchIndices(*map_kd_tree_, point, config_.knn_rad);

      // TODO (achains): Move to config?
      int32_t component_inner_min_knn = 3;
      bool enough_neighbours = neighbours_idx.size() > component_inner_min_knn;
      if (enough_neighbours) {
        component_metric_statistic.push_back(metric(map_kd_tree_->getPointsMatrixMap()(Eigen::all, neighbours_idx)));
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
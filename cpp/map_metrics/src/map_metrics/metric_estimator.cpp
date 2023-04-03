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

namespace map_metrics {
class MetricsEstimator::Impl {
 public:
  Impl(Eigen::Matrix3Xd const& points, Config const& config);

  double MME();

  double MPV();

 private:
  std::unique_ptr<cilantro::KDTree3d<>> map_kd_tree_;
  Config config_;

  double baseMetricEstimator(double (*metric)(Eigen::Matrix3Xd const& points));
};

MetricsEstimator::MetricsEstimator(Eigen::Matrix3Xd const& points, map_metrics::Config const& config)
    : impl_(new Impl(points, config)) {}

MetricsEstimator::~MetricsEstimator() = default;

MetricsEstimator::MetricsEstimator(map_metrics::MetricsEstimator&& op) noexcept = default;

MetricsEstimator& MetricsEstimator::operator=(map_metrics::MetricsEstimator&& op) noexcept = default;

double MetricsEstimator::MME() { return impl_->MME(); }

double MetricsEstimator::MPV() { return impl_->MPV(); }

MetricsEstimator::Impl::Impl(Eigen::Matrix3Xd const& points, map_metrics::Config const& config)
    : map_kd_tree_(std::make_unique<cilantro::KDTree3d<>>(points)), config_(config) {}

double MetricsEstimator::Impl::MME() { return baseMetricEstimator(computePointsEntropy); }

double MetricsEstimator::Impl::MPV() { return baseMetricEstimator(computePointsVariance); }

double MetricsEstimator::Impl::baseMetricEstimator(double (*metric)(Eigen::Matrix3Xd const& points)) {
  std::vector<double> metric_statistic;
  for (auto const& point : map_kd_tree_->getPointsMatrixMap().colwise()) {
    auto neighbours_idx = getRadiusSearchIndices(*map_kd_tree_, point, config_.knn_rad);

    bool enough_neighbours = neighbours_idx.size() > config_.min_knn;
    if (enough_neighbours) {
      metric_statistic.push_back(metric(map_kd_tree_->getPointsMatrixMap()(Eigen::all, neighbours_idx)));
    }
  }

  return Eigen::Map<Eigen::VectorXd>(metric_statistic.data(), metric_statistic.size()).mean();
}

}  // namespace map_metrics
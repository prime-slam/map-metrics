/**
 * Copyright 2022 prime-slam
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "map_metrics/metric_estimator.h"

#include <cilantro/core/kd_tree.hpp>

#include "point_statistics.h"

namespace map_metrics {
class MetricEstimator::Impl {
 public:
  Impl(Eigen::Ref<const Eigen::Matrix4d> const& points, Config const& config);

  double MME();

  double MPV();

 private:
  std::unique_ptr<cilantro::KDTree3d<>> map_kd_tree_;
  Config config_;

  double baseMetricEstimation(double (*metric)(cilantro::VectorSet3d const& points));
};

MetricEstimator::Impl::Impl(Eigen::Ref<const Eigen::Matrix4d> const& points, Config const& config) {}

double MetricEstimator::Impl::MME() { return baseMetricEstimation(computePointsEntropy); }

double MetricEstimator::Impl::MPV() { return baseMetricEstimation(computePointsVariance); }

double MetricEstimator::Impl::baseMetricEstimation(double (*metric)(cilantro::VectorSet3d const& points)) {
  std::vector<double> metric_statistic;
  for (auto point : map_kd_tree_->getPointsMatrixMap().colwise()) {
    cilantro::NeighborSet<double> nn = map_kd_tree_->radiusSearch(point, std::pow(config_.knn_rad, 2.0));
    Eigen::ArrayXi mask = Eigen::ArrayXi::Zero(map_kd_tree_->getPointsMatrixMap().cols());
    for (auto neighbour : nn){
      mask[neighbour.index] = 1;
    }

    if (mask.count() > config_.min_knn){
      // TODO
    }
  }
}

}  // namespace map_metrics
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
#include "point_statistics.h"

#include <cassert>
#include <cmath>

namespace map_metrics {

Eigen::MatrixX3d findCovariance(Eigen::Ref<const Eigen::Matrix3Xd> const& points) {
  Eigen::MatrixX3d centered = points.rowwise() - points.colwise().mean();
  return (centered.adjoint() * centered) / (static_cast<double>(points.rows()) - 1.0);
}

double computePointsVariance(Eigen::Ref<const Eigen::Matrix3Xd> const& points) {
  Eigen::VectorXd eigenvalues = findCovariance(points).eigenvalues().real();
  return eigenvalues.minCoeff();
}

double computePointsEntropy(Eigen::Ref<const Eigen::Matrix3Xd> const& points) {
  double det = 2.0 * M_PI * M_E * findCovariance(points).determinant();
  assert(det > 0 && "Determinant of covariance matrix has to be non-negative");
  return 0.5 * std::log(det);
}
}  // namespace map_metrics
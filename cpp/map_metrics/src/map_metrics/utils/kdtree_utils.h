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
#pragma once

#include <vector>

#include <cilantro/core/kd_tree.hpp>

namespace map_metrics {
Eigen::MatrixX3d transformPointIdxToMatrix(Eigen::Ref<const Eigen::Matrix3Xd> const& points,
                                           std::vector<Eigen::Index> const& idx);

std::vector<Eigen::Index> getRadiusSearchIndices(cilantro::KDTree3d<> const& tree,
                                                 Eigen::Ref<const Eigen::Vector3d> const& query, double radius);
}  // namespace map_metrics
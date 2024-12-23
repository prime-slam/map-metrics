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
#include "map_metrics/map_tree.h"

#include "cilantro/core/kd_tree.hpp"

namespace map_metrics {

class MapTree::MapTreeImpl {
 public:
  MapTreeImpl(Eigen::Matrix3Xd const& points, double knn_rad);

  std::vector<std::vector<Eigen::Index>> const& getMapNeighbours() const;

  std::vector<std::vector<Eigen::Index>> getNeighboursByComponent(Eigen::Matrix3Xd const& point_component) const;

  Eigen::Matrix3Xd getMapPoints(std::vector<Eigen::Index> const& point_indices) const;

 private:
  double knn_rad_;
  Eigen::Matrix3Xd const& points_;
  cilantro::KDTree3d<> kd_tree_;
  std::vector<std::vector<Eigen::Index>> map_neighbours_;

  std::vector<Eigen::Index> getRadiusSearchIndices(Eigen::Vector3d const& query) const;
};

MapTree::MapTree(Eigen::Matrix3Xd const& points, double knn_rad) : impl_(new MapTreeImpl(points, knn_rad)) {}

MapTree::~MapTree() = default;

MapTree::MapTree(MapTree&& op) noexcept = default;

MapTree& MapTree::operator=(MapTree&& op) noexcept = default;

std::vector<std::vector<Eigen::Index>> const& MapTree::getMapNeighbours() const { return impl_->getMapNeighbours(); }

std::vector<std::vector<Eigen::Index>> MapTree::getNeighboursByComponent(
    Eigen::Matrix3Xd const& point_component) const {
  return impl_->getNeighboursByComponent(point_component);
}

Eigen::Matrix3Xd MapTree::getMapPoints(std::vector<Eigen::Index> const& point_indices) const {
  return impl_->getMapPoints(point_indices);
}

MapTree::MapTreeImpl::MapTreeImpl(Eigen::Matrix3Xd const& points, double knn_rad)
    : knn_rad_(knn_rad), points_(points), kd_tree_(points_), map_neighbours_(getNeighboursByComponent(points)) {}

std::vector<std::vector<Eigen::Index>> const& MapTree::MapTreeImpl::getMapNeighbours() const { return map_neighbours_; }

std::vector<std::vector<Eigen::Index>> MapTree::MapTreeImpl::getNeighboursByComponent(
    Eigen::Matrix3Xd const& point_component) const {
  std::vector<std::vector<Eigen::Index>> neighbours;
  for (auto const& point : point_component.colwise()) {
    neighbours.push_back(getRadiusSearchIndices(point));
  }
  return neighbours;
}

Eigen::Matrix3Xd MapTree::MapTreeImpl::getMapPoints(std::vector<Eigen::Index> const& point_indices) const {
  return points_(Eigen::all, point_indices);
}

std::vector<Eigen::Index> MapTree::MapTreeImpl::getRadiusSearchIndices(Eigen::Vector3d const& query) const {
  cilantro::NeighborSet<double> nn = kd_tree_.radiusSearch(query, pow(knn_rad_, 2.0));
  std::vector<Eigen::Index> idx_vector(nn.size());
  std::transform(nn.begin(), nn.end(), idx_vector.begin(), [](auto n) { return n.index; });
  return idx_vector;
}

}  // namespace map_metrics
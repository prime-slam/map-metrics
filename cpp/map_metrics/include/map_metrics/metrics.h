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
#ifndef MAP_METRICS_METRICS_H
#define MAP_METRICS_METRICS_H
#include "map_metrics/map_tree.h"

namespace map_metrics {
double MME(MapTree const& map_tree, int min_component_size);

double MPV(MapTree const& map_tree, int min_component_size);

double MOM(MapTree const& map_tree, int min_component_size, std::vector<Eigen::Matrix3Xd> const& orthogonal_subset);
}  // namespace map_metrics
#endif  // MAP_METRICS_METRICS_H

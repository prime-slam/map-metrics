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
#include "cloud_utils.h"

#include <pybind11/eigen.h>

#include "map_metrics/utils/cloud_utils.h"

namespace map_metrics {
void pybindCloudUtils(py::module& m) {
  py::module m_cloud_utils = m.def_submodule("cloud_utils", "Orthogonal extraction, map composition utils");

  pybindAggregateMap(m_cloud_utils);
  pybindFindOrthogonalSubset(m_cloud_utils);
}

void pybindAggregateMap(py::module& m) {
  m.def("aggregate_map", &aggregateMap, py::arg("point_sequence"), py::arg("poses"));
}

void pybindFindOrthogonalSubset(py::module& m) {
  m.def("find_orthogonal_subset", &findOrthogonalSubset, py::arg("points"), py::arg("config"));
}
}  // namespace map_metrics
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
#include "config.h"

#include "map_metrics/config.h"

namespace map_metrics {
void pybindConfig(py::module& m) {
  py::module_ m_config = m.def_submodule("config", "Predefined config class");

  py::class_<Config>(m_config, "Config")
      .def(py::init<int, double, int, int>(), py::arg("min_knn") = 5, py::arg("knn_rad") = 1.0, py::arg("max_nn") = 30,
           py::arg("min_clust_size") = 5);
}
}  // namespace map_metrics
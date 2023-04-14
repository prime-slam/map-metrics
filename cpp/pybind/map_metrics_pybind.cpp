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
#include "pybind11/pybind11.h"

#include "config.h"
#include "map_tree.h"
#include "metrics.h"
#include "utils/cloud_utils.h"

namespace map_metrics {
PYBIND11_MODULE(pybind, m) {
  m.doc() = "Pybind metrics module";

  pybindConfig(m);
  pybindMapTree(m);
  pybindCloudUtils(m);
  pybindMetrics(m);
}
}  // namespace map_metrics
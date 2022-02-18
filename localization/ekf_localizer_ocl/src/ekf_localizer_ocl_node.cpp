// Copyright 2021 Tier IV inc. All rights reserved.
//
// This file is based ekf_localizer_node.cpp with using EKFLocalizerOCL.
//
// This file is also licensed under the Apache License, Version 2.0.
//
// ORIGINAL LICENSE
//
// Copyright 2018-2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ekf_localizer_ocl/ekf_localizer_ocl.hpp"

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  auto node = std::make_shared<EKFLocalizerOCL>("ekf_localizer_ocl", node_options);

  rclcpp::spin(node);

  return 0;
}

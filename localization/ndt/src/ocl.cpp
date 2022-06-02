// Copyright 2021 Tier IV inc. All rights reserved.
//
// This source includes PCLOMP NDT, with naming changes for OpenCL version.
//
// This class is also licensed under the Apache License, Version 2.0.
//
// ORIGINAL LICENSE
//
// Copyright 2015-2019 Autoware Foundation
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

#include "ndt/ocl.hpp"

#include "ndt/impl/ocl.hpp"

template class NormalDistributionsTransformOCL<pcl::PointXYZ, pcl::PointXYZ>;
template class NormalDistributionsTransformOCL<pcl::PointXYZI, pcl::PointXYZI>;

// Copyright 2020 Autonomous Robots Lab, University of Nevada, Reno

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CUPOCH_CONVERSIONS__CUPOCH_CONVERSIONS_HPP_
#define CUPOCH_CONVERSIONS__CUPOCH_CONVERSIONS_HPP_

// cupoch
#include "cupoch/cupoch.h"

// ROS
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// Eigen
#include <Eigen/Dense>

// C++
#include <string>

#include "cupoch_conversions/visibility_control.h"

namespace cupoch_conversions
{
  /**
   * @brief Copy data from a cupoch::geometry::PointCloud to a sensor_msgs::msg::PointCloud2
   *
   * @param pointcloud Reference to the cupoch PointCloud
   * @param ros_pc2 Reference to the sensor_msgs PointCloud2
   * @param frame_id The string to be placed in the frame_id of the PointCloud2
   */
  void cupochToRos(std::shared_ptr<cupoch::geometry::PointCloud> &pointcloud, sensor_msgs::msg::PointCloud2 &ros_pc2,
                   std::string frame_id = "cupoch_pointcloud");

  /**
   * @brief Copy data from a sensor_msgs::msg::PointCloud2 to a cupoch::geometry::PointCloud
   *
   * @param ros_pc2 Reference to the sensor_msgs PointCloud2
   * @param o3d_pc Reference to the cupoch PointCloud
   * @param skip_colors If true, only xyz fields will be copied
   */
  void rosToCupoch(const sensor_msgs::msg::PointCloud2::SharedPtr &ros_pc2, std::shared_ptr<cupoch::geometry::PointCloud> &o3d_pc,
                   bool skip_colors = false);
} // namespace cupoch_conversions

#endif // CUPOCH_CONVERSIONS__CUPOCH_CONVERSIONS_HPP_

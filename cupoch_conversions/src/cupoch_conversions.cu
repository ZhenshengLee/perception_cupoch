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

// C++
#include <memory>
#include <string>

#include "cupoch_conversions/cupoch_conversions.hpp"

namespace cupoch_conversions
{
  // d2h: get points
  void cupochToRos(std::shared_ptr<cupoch::geometry::PointCloud> &pointcloud, sensor_msgs::msg::PointCloud2 &ros_pc2, std::string frame_id)
  {
    // d2h
    auto pointcloud_points_host = pointcloud->GetPoints();
    auto pointcloud_colors_host = pointcloud->GetColors();

    sensor_msgs::PointCloud2Modifier modifier(ros_pc2);
    if (pointcloud->HasColors())
    {
      modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    }
    else
    {
      modifier.setPointCloud2FieldsByString(1, "xyz");
    }
    modifier.resize(pointcloud_points_host.size());
    ros_pc2.header.frame_id = frame_id;
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(ros_pc2, "x");
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(ros_pc2, "y");
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(ros_pc2, "z");

    if (pointcloud->HasColors())
    {
      sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_r(ros_pc2, "r");
      sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_g(ros_pc2, "g");
      sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_b(ros_pc2, "b");
      for (size_t i = 0; i < pointcloud_points_host.size();
           i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b)
      {
        const Eigen::Vector3f &point = pointcloud_points_host[i];
        const Eigen::Vector3f &color = pointcloud_colors_host[i];
        *ros_pc2_x = point(0);
        *ros_pc2_y = point(1);
        *ros_pc2_z = point(2);
        *ros_pc2_r = (uint8_t)(255 * color(0));
        *ros_pc2_g = (uint8_t)(255 * color(1));
        *ros_pc2_b = (uint8_t)(255 * color(2));
      }
    }
    else
    {
      for (size_t i = 0; i < pointcloud_points_host.size(); i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
      {
        const Eigen::Vector3f &point = pointcloud_points_host[i];
        *ros_pc2_x = point(0);
        *ros_pc2_y = point(1);
        *ros_pc2_z = point(2);
      }
    }
  }

  // gpu不能随意push_back
  // h2d
  void rosToCupoch(const sensor_msgs::msg::PointCloud2::SharedPtr &ros_pc2, std::shared_ptr<cupoch::geometry::PointCloud> &cupoch_pc, bool skip_colors)
  {
    // host
    thrust::host_vector<Eigen::Vector3f> cupoch_pc_points_host;
    thrust::host_vector<Eigen::Vector3f> cupoch_pc_colors_host;

    sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_x(*ros_pc2, "x");
    sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_y(*ros_pc2, "y");
    sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_z(*ros_pc2, "z");
    cupoch_pc_points_host.reserve(ros_pc2->height * ros_pc2->width);
    if (ros_pc2->fields.size() == 3 || skip_colors == true)
    {
      for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
      {
        cupoch_pc_points_host.push_back(Eigen::Vector3f(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
      }
    }
    else
    {
      cupoch_pc_colors_host.reserve(ros_pc2->height * ros_pc2->width);
      if (ros_pc2->fields[3].name == "rgb")
      {
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_r(*ros_pc2, "r");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_g(*ros_pc2, "g");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_b(*ros_pc2, "b");

        for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
             ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b)
        {
          cupoch_pc_points_host.push_back(Eigen::Vector3f(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
          cupoch_pc_colors_host.push_back(Eigen::Vector3f(((int)(*ros_pc2_r)) / 255.0, ((int)(*ros_pc2_g)) / 255.0,
                                                   ((int)(*ros_pc2_b)) / 255.0));
        }
      }
      else if (ros_pc2->fields[3].name == "intensity")
      {
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_i(*ros_pc2, "intensity");
        for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
             ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_i)
        {
          cupoch_pc_points_host.push_back(Eigen::Vector3f(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
          cupoch_pc_colors_host.push_back(Eigen::Vector3f(*ros_pc2_i, *ros_pc2_i, *ros_pc2_i));
        }
      }
    }
    // h2d
    cupoch_pc->SetPoints(cupoch_pc_points_host);
    cupoch_pc->SetColors(cupoch_pc_colors_host);
  }

//   zs@zs-dell:~$ rostopic echo /velodyne_points
// header:
//   seq: 7302
//   stamp:
//     secs: 1607570362
//     nsecs: 406008720
//   frame_id: "velodyne"
// height: 1
// width: 25556
// fields:
//   -
//     name: "x"
//     offset: 0
//     datatype: 7
//     count: 1
//   -
//     name: "y"
//     offset: 4
//     datatype: 7
//     count: 1
//   -
//     name: "z"
//     offset: 8
//     datatype: 7
//     count: 1
//   -
//     name: "intensity"
//     offset: 12
//     datatype: 7
//     count: 1
//   -
//     name: "ring"
//     offset: 16
//     datatype: 4
//     count: 1
//   -
//     name: "time"
//     offset: 18
//     datatype: 7
//     count: 1
// is_bigendian: False
// point_step: 22
// row_step: 0
// data: [13, 38, 106, 192, 89

} // namespace cupoch_conversions

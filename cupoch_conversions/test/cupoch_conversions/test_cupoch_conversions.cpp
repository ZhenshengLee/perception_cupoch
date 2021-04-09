// GTest
#include <gtest/gtest.h>

//cupoch_conversions
#include "cupoch_conversions/cupoch_conversions.hpp"

// cupoch
#include <cupoch/cupoch.h>

// ROS
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// C++
#include <memory>

// ROS
#include "rclcpp/rclcpp.hpp"

using namespace cupoch;
using namespace Eigen;

TEST(ConversionFunctions, cupochToRos2_uncolored)
{
  auto cupoch_pc = std::make_shared<cupoch::geometry::PointCloud>();
  thrust::host_vector<Eigen::Vector3f> cupoch_pc_points_host;
  for (int i = 0; i < 5; ++i)
  {
    cupoch_pc_points_host.push_back(Eigen::Vector3f((0.5 * i), (i * i), (10.5 * i)));
  }
  cupoch_pc->SetPoints(cupoch_pc_points_host);

  sensor_msgs::msg::PointCloud2 ros_pc2;
  cupoch_conversions::cupochToRos(cupoch_pc, ros_pc2, "cupoch_frame");
  EXPECT_EQ(ros_pc2.height * ros_pc2.width, cupoch_pc->GetPoints().size());
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(ros_pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(ros_pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(ros_pc2, "z");
  for (int i = 0; i < 5; i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
  {
    const Eigen::Vector3f &point = cupoch_pc->GetPoints()[i];
    EXPECT_EQ(*ros_pc2_x, (0.5 * i));
    EXPECT_EQ(*ros_pc2_y, (i * i));
    EXPECT_EQ(*ros_pc2_z, (10.5 * i));
  }
}

TEST(ConversionFunctions, cupochToRos2_colored)
{
  auto cupoch_pc = std::make_shared<cupoch::geometry::PointCloud>();
  thrust::host_vector<Eigen::Vector3f> cupoch_pc_points_host;
  thrust::host_vector<Eigen::Vector3f> cupoch_pc_colors_host;
  for (int i = 0; i < 5; ++i)
  {
    cupoch_pc_points_host.push_back(Eigen::Vector3f((0.5 * i), (i * i), (10.5 * i)));
    cupoch_pc_colors_host.push_back(Eigen::Vector3f(2 * i / 255.0, 5 * i / 255.0, 10 * i / 255.0));
  }
  cupoch_pc->SetPoints(cupoch_pc_points_host);
  cupoch_pc->SetColors(cupoch_pc_colors_host);

  EXPECT_EQ(cupoch_pc_points_host.size(), cupoch_pc_colors_host.size());
  EXPECT_EQ(false, cupoch_pc->IsEmpty());
  EXPECT_EQ(true, cupoch_pc->HasColors());

  sensor_msgs::msg::PointCloud2 ros_pc2;
  cupoch_conversions::cupochToRos(cupoch_pc, ros_pc2, "cupoch_frame");
  EXPECT_EQ(ros_pc2.height * ros_pc2.width, cupoch_pc->GetPoints().size());
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(ros_pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(ros_pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(ros_pc2, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_r(ros_pc2, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_g(ros_pc2, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_b(ros_pc2, "b");
  for (int i = 0; i < 5; i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b)
  {
    const Eigen::Vector3f &point = cupoch_pc->GetPoints()[i];
    EXPECT_EQ(*ros_pc2_x, (0.5 * i));
    EXPECT_EQ(*ros_pc2_y, (i * i));
    EXPECT_EQ(*ros_pc2_z, (10.5 * i));
    const Eigen::Vector3f &color = cupoch_pc->GetPoints()[i];
    EXPECT_EQ(*ros_pc2_r, (int)(2 * i));
    EXPECT_EQ(*ros_pc2_g, (int)(5 * i));
    EXPECT_EQ(*ros_pc2_b, (int)(10 * i));
  }
}

TEST(ConversionFunctions, rosToCupoch_uncolored)
{
  sensor_msgs::msg::PointCloud2 ros_pc2;
  ros_pc2.header.frame_id = "ros";
  ros_pc2.height = 1;
  ros_pc2.width = 5;
  ros_pc2.is_bigendian = false;
  ros_pc2.is_dense = true;
  sensor_msgs::PointCloud2Modifier modifier(ros_pc2);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(5 * 1);
  sensor_msgs::PointCloud2Iterator<float> mod_x(ros_pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> mod_y(ros_pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> mod_z(ros_pc2, "z");

  for (int i = 0; i < 5; ++i, ++mod_x, ++mod_y, ++mod_z)
  {
    *mod_x = 0.5 * i;
    *mod_y = i * i;
    *mod_z = 10.5 * i;
  }

  const sensor_msgs::msg::PointCloud2::SharedPtr &ros_pc2_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(ros_pc2);
  auto cupoch_pc = std::make_shared<cupoch::geometry::PointCloud>();
  cupoch_conversions::rosToCupoch(ros_pc2_ptr, cupoch_pc);

  auto cupoch_pc_points_host = cupoch_pc->GetPoints();
  EXPECT_EQ(ros_pc2_ptr->height * ros_pc2_ptr->width, cupoch_pc_points_host.size());
  EXPECT_EQ(cupoch_pc->HasColors(), false);
  for (unsigned int i = 0; i < 5; i++)
  {
    const Eigen::Vector3f &point = cupoch_pc_points_host[i];
    EXPECT_EQ(point(0), 0.5 * i);
    EXPECT_EQ(point(1), i * i);
    EXPECT_EQ(point(2), 10.5 * i);
  }
}

TEST(ConversionFunctions, rosToCupoch_colored)
{
  sensor_msgs::msg::PointCloud2 ros_pc2;
  ros_pc2.header.frame_id = "ros";
  ros_pc2.height = 1;
  ros_pc2.width = 5;
  ros_pc2.is_bigendian = false;
  ros_pc2.is_dense = true;
  sensor_msgs::PointCloud2Modifier modifier(ros_pc2);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(5 * 1);

  sensor_msgs::PointCloud2Iterator<float> mod_x(ros_pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> mod_y(ros_pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> mod_z(ros_pc2, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> mod_r(ros_pc2, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> mod_g(ros_pc2, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> mod_b(ros_pc2, "b");

  for (int i = 0; i < 5; ++i, ++mod_x, ++mod_y, ++mod_z, ++mod_r, ++mod_g, ++mod_b)
  {
    *mod_x = 0.5 * i;
    *mod_y = i * i;
    *mod_z = 10.5 * i;
    *mod_r = 2 * i;
    *mod_g = 5 * i;
    *mod_b = 10 * i;
  }

  const sensor_msgs::msg::PointCloud2::SharedPtr &ros_pc2_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(ros_pc2);
  auto cupoch_pc = std::make_shared<cupoch::geometry::PointCloud>();
  cupoch_conversions::rosToCupoch(ros_pc2_ptr, cupoch_pc);

  auto cupoch_pc_points_host = cupoch_pc->GetPoints();
  auto cupoch_pc_colors_host = cupoch_pc->GetColors();
  EXPECT_EQ(ros_pc2_ptr->height * ros_pc2_ptr->width, cupoch_pc_points_host.size());
  EXPECT_EQ(cupoch_pc->HasColors(), true);
  for (unsigned int i = 0; i < 5; i++)
  {
    const Eigen::Vector3f &point = cupoch_pc_points_host[i];
    EXPECT_FLOAT_EQ(point(0), 0.5 * i);
    EXPECT_FLOAT_EQ(point(1), i * i);
    EXPECT_FLOAT_EQ(point(2), 10.5 * i);
    const Eigen::Vector3f &color = cupoch_pc_colors_host[i];
    EXPECT_FLOAT_EQ(color(0), 2 * i / 255.0);
    EXPECT_FLOAT_EQ(color(1), 5 * i / 255.0);
    EXPECT_FLOAT_EQ(color(2), 10 * i / 255.0);
  }
}

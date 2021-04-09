# cupoch_conversions

This package provides functions that can convert pointclouds from ROS to CUPOCH and vice-versa.

cupoch默认的ros-io接口比较难用, 这里用通用的方法, 根据open3d_conversion重构而成

the ros-io interface of cupoch is tough, so this package is generated according to open3d_conversion package.

注意: ros point_cloud2是Float32的点云数据(目前只考虑这种)和uint8的rgb数据, 而cupoch::Pointcloud是vector3f, 浮点数据, 所以转化过程中可能有精度损失

## Dependencies

* Eigen3
### cupoch_conversions

* In case you are building this package from source, time taken for the conversion functions will be much larger if it is not built in `Release` mode.

## Usage

There are two functions provided in this library:

```cpp
void cupoch_ros::cupochToRos(const cupoch::geometry::PointCloud& pointcloud, sensor_msgs::msg::PointCloud2& ros_pc2, std::string frame_id = "cupoch_pointcloud");

void cupoch_ros::rosToCupoch(const sensor_msgs::msg::PointCloud2::SharedPtr& ros_pc2, cupoch::geometry::PointCloud& o3d_pc, bool skip_colors=false);
```

* As CUPOCH pointclouds only contain `points`, `colors` and `normals`, the interface currently supports XYZ, XYZRGB pointclouds. XYZI pointclouds are handled by placing the `intensity` value in the `colors_`.
* On creating a ROS pointcloud from an CUPOCH pointcloud, the user is expected to set the timestamp in the header and pass the `frame_id` to the conversion function.

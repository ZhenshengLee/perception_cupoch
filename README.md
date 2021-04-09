# perception_cupoch

This package provides functions that can convert pointclouds from ROS to cupoch and vice-versa.

## Dependencies

* Eigen3
* cupoch

## System Requirements

* Ubuntu 18.04+: GCC 5+
* cuda10.2+

## Installation

### cupoch

* Instructions to setup cupoch can be found [here](https://github.com/neka-nat/cupoch).

### cupoch_conversions

* In case you are building this package from source, time taken for the conversion functions will be much larger if it is not built in `Release` mode.

## Build

in `cupoch_conversions/cmake/ga_build_common.cmake`, set the `CUPOCH_ROOT` correctly.

```sh
export GPUAC_COMPILE_WITH_CUDA=1
catkin_make -j5 -DCMAKE_BUILD_TYPE=Release
```

## test

### Unit Test

```sh
cd ./devel/lib/cupoch_conversions
./cupoch_test
./cupoch_conversions_test
```

### ROS node test

```sh
roslaunch cupoch_conversions cupoch_conversions_test_node.launch
roslaunch cupoch_conversions pub_pcd.launch
```

## API Usage

There are two functions provided in this library:

```cpp
void cupochToRos(std::shared_ptr<cupoch::geometry::PointCloud> &pointcloud, sensor_msgs::PointCloud2 &ros_pc2,
                   std::string frame_id = "cupoch_pointcloud");

void rosToCupoch(const sensor_msgs::PointCloud2ConstPtr &ros_pc2, std::shared_ptr<cupoch::geometry::PointCloud> &o3d_pc,
                   bool skip_colors = false);
```

* As cupoch pointclouds only contain `points`, `colors` and `normals`, the interface currently supports XYZ, XYZRGB pointclouds. XYZI pointclouds are handled by placing the `intensity` value in the `colors_`.
* On creating a ROS pointcloud from an cupoch pointcloud, the user is expected to set the timestamp in the header and pass the `frame_id` to the conversion function.

## Documentation

Documentation can be generated using Doxygen and the configuration file by executing  `doxygen Doxyfile` in the package.

## Contact

Feel free to contact us for any questions:

* [ZhenshengLi](mailto:lzs_1993@qq.com)

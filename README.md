# perception_cupoch

This package provides functions that can convert pointclouds from ROS to cupoch and vice-versa.

## Dependencies

* Eigen3-(currently 3.3.7)
* cupoch-(currently 0.1.9.1) 

## System Requirements

* Ubuntu 18.04+: GCC 5+
* cuda10.2+

## Installation

### eigen3

upgrade eigen to 3.3.7

```sh
# 1804
sudo apt install libopenblas-dev freeglut3-dev libglew-dev libadolc-dev libcholmod3 libumfpack5 libmetis-dev libspqr2 libsparsehash-dev libmpfr-dev libmpfrc++-dev libfftw3-dev
# jetson
sudo apt install libopenblas-dev freeglut3-dev libglew-dev libadolc-dev libcholmod3.0.6 libumfpack5.7.1 libmetis-dev libspqr2.0.2 libsparsehash-dev libmpfr-dev libmpfrc++-dev libfftw3-dev

cmake .. -DCMAKE_INSTALL_PREFIX=/usr
sudo make install
```

### cupoch

~~* Instructions to setup cupoch can be found [here](https://github.com/neka-nat/cupoch).~~

Because of issues, you can use the custom version of cupoch, naming [cupoch-fat](https://github.com/ZhenshengLee/cupoch-fat).

And follow [this guide](./cupoch_conversions/docs/tutorial.md) to install cupoch to the path of `/opt/cupoch/`.

### cupoch_conversions

* In case you are building this package from source, time taken for the conversion functions will be much larger if it is not built in `Release` mode.

## Build

in `cupoch_conversions/cmake/ga_build_common.cmake`, set the `CUPOCH_ROOT` correctly.

```sh
export GPUAC_COMPILE_WITH_CUDA=1
catkin_make -j5 -DCMAKE_BUILD_TYPE=Release
```

## (Optional) Performance tunning for jetson

**NOTE:** You should set power management policy for cpu and gpu to get the best performance of `cudaMemCpy`.

### cpu frequency scaling

refer to [this guide from stackoverflow](https://askubuntu.com/questions/523640/how-i-can-disable-cpu-frequency-scaling-and-set-the-system-to-performance) to disable cpu scaling.

```sh
sudo apt-get install cpufrequtils
# to get the highest freq
sudo cpufreq-set -g performance
```

```sh
# to get the default mode powersave
sudo cpufreq-set -g powersave
```

### gpu frequency scaling

refer to [this guide](https://elinux.org/Jetson/AGX_Xavier_Performance_Governor) for more details.

```sh
# If not, please set CONFIG_DEVFREQ_GOV_PERFORMANCE=y and update the kernel
zcat /proc/config.gz | grep CONFIG_DEVFREQ_GOV_PERFORMANCE
# Run below command on board as root to keep GPU alive (not suspend)
echo 0 > /sys/devices/17000000.gv11b/railgate_enable
# Enable performance governor for GPU
cat /sys/class/devfreq/17000000.gv11b/available_governors
echo performance > /sys/class/devfreq/17000000.gv11b/governor
cat /sys/class/devfreq/17000000.gv11b/governor
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
# t1
roslaunch cupoch_conversions cupoch_conversions_test_node.launch
# t2
rosrun plotjuggler plotjuggler
# plot the topic /cupoch/ros2cupoch
```

### performance benchmark

rmm benchmark.

```sh
cd ./devel/lib/cupoch_conversions
./RANDOM_ALLOCATIONS_BENCH  -n 1000 -m 1024
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

## Example

Please refer to this repo [ga_points_downsampler](https://github.com/ZhenshengLee/ga_points_downsampler) which is a ros node to downsample pointclouds with cupoch.

## Documentation

Documentation can be generated using Doxygen and the configuration file by executing  `doxygen Doxyfile` in the package.

## Contact

Feel free to contact us for any questions:

* [ZhenshengLi](mailto:lzs_1993@qq.com)

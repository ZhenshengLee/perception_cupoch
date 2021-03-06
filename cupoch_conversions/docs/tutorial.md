# tutorial of perception_cupoch

## Tested platform

- x86 PC
  - ubuntu-20.04
  - ROS Noetic with apt
  - GPU Driver-460.91.03
  - Cuda-11.0
  - cupoch-0.1.9.1
- Jetson xavier
  - Jetpack-4.4.1(L4T 32.4.4)
  - Ubuntu-18.04
  - ROS-Melodic with apt
  - Cuda-10.2
- Jetson container on x86 PC
  - Jetpack 4.4.1

## install

### install cupoch

install custom version of cupoch-fat, which is based on cupoch 0.1.9.1.

```sh
sudo apt install nasm
sudo apt-get --yes install xorg-dev libglu1-mesa-dev libgl1-mesa-glx libglew-dev libglfw3-dev libeigen3-dev libpng-dev libpng16-16  libsdl2-dev python-dev python-tk python3-dev python3-tk  libtbb-dev  libglu1-mesa-dev  libc++-7-dev  libc++abi-7-dev  ninja-build  libxi-dev
sudo apt install libfmt-dev pybind11-dev libqhull-dev libglfw3-dev liblapacke-dev
```

```sh

git clone https://github.com/ZhenshengLee/cupoch-fat.git
cd ./cupoch-fat
mkdir build
cd ./build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/cupoch/cupoch -DBUILD_UNIT_TESTS=OFF -DBUILD_PYBIND11=OFF -DBUILD_PYTHON_MODULE=OFF
make -j6
sudo make install
```

## compile

following guide in [README of cupoch_conversions](../../README.md)

## test

following guide in [README of cupoch_conversions](../../README.md)

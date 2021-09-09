- replace boost.h in `/usr/include/pcl-1.8/pcl/io/boost.h` to fix cuda compiling issue.

```sh
sudo rm /usr/include/pcl-1.8/pcl/io/boost.h
sudo cp ./boost.h /usr/include/pcl-1.8/pcl/io/boost.h
```


# ROS Wrapper around MSDI: Multi-spectral dataset

This is meant as simple, minimal example of how to process multi-spectal dataset.

# 1. Installation

catkin_ws

``` shell
 mkdir -p ~/catkin_ws/src
 cd ~/catkin_ws/src
 git clone git@github.com:weichnn/bag_extractor.git
 cd ~/catkin_ws/
 catkin_make

```

# 2 Usage

## process rosbag

### extract synced RGB+LWIR+Depth data

``` shell
roslaunch msdi_ros extract_msdi_rld.launch
```

### extract synced RGB+LWIR data

``` shell
roslaunch msdi_ros extract_msdi_rl.launch
```

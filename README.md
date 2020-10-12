
# ROS Wrapper around MSDI: Multi-spectral dataset

This is meant as simple, minimal example of how to process multi-spectal dataset.

# 1. Installation

catkin_ws
```
	mkdir -p ~/catkin_ws/src
	cd ~/catkin_ws/
	catkin_make
	cd src
	git clone 
	
```

# 2 Usage

## process rosbag

### extract synced RGB+LWIR+Depth data

```
roslaunch msdi_ros extract_msdi_rld_dataset.launch
```

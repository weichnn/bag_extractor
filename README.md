
# ROS Wrapper for Multi-spectral Dataset

This is meant as simple, minimal example of how to process [multi-spectal dataset](https://github.com/NGCLAB/multi-spectral-dataset) and how to generate dense depth images.

## 1. Prerequisites

### 1.1 ROS and Python

There are two ways to install ROS:

1. The instruction in the link: <http://wiki.ros.org/ROS/Installation>
2. The shell script in this repository: <https://github.com/weichnn/env>

This code have been tested with ROS Melodic under Ubuntu 18.04.

### 1.2 OpenCV

Install with

``` shell
sudo apt-get install libopencv-dev ros-<distro>-cv-bridge
```

### 1.3 Matlab (optional)

Due to the unique measuring principle of depth cameras, depth information at the edges of objects is easily missing. The presence of parallax between the standard camera and the depth camera also leads to missing depth information at the object's boundary. Therefore, to obtain a dense depth map from the standard camera's viewpoint, the projected depth map needs to be inpainted with color images to fill the hole.

The used program is written in matlab. Therefore, to get dense depth images, the MATLAB is required.
Get MATLAB for linux in <https://www.mathworks.com/products/matlab.html>

## 2 Build 

catkin_ws

``` shell
 mkdir -p ~/catkin_ws/src
 cd ~/catkin_ws/src
 git clone git@github.com:weichnn/bag_extractor.git
 cd bag_extractor/rgbdUtils
 ./build.sh
 cd ~/catkin_ws/
 catkin_make
```

## 3 Usage

### 3.1 only extract data from rosbag

- extract synced RGB+LWIR+Depth data

``` shell
roslaunch msdi_ros extract_msdi_rld.launch
```

- extract synced RGB+LWIR data

``` shell
roslaunch msdi_ros extract_msdi_rl.launch
```

### 3.2 extract and get dense depth images

``` shell
./processBags.sh path_to_data_folder path_to_output_folder ros_launchfile_name
```

e.g.,

``` shell
./processBags.sh /media/ubuntu/HDDData/multi-spectral-dataset/
 /media/ubuntu/nvmeData/multi-spectral-dataset/
 extract_msdi_rl.launch
```



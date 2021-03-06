# Utilities for Multi-spectral Dataset

This is meant as simple, minimal example of how to extract data from [multi-spectal dataset](https://github.com/NGCLAB/multi-spectral-dataset) bags and how to generate dense depth images in the standard camera's view.

The repository consists of two parts, the ROS bag extraction program and the depth image processing program. The ROS bag extraction program is mainly responsible for extracting the BOS bag files' data and generating the corresponding data directory. The depth image processing program is used to obtain a complete dense depth image in the standard camera view using multiple view transformation and inpaint algorithms.

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
 source devel/setup.bash 
```

## 3 Usage

### 3.1 only extract data from rosbag

1. run roslaunch

    - extract synced RGB+LWIR+Depth data

    ``` shell
    roslaunch msdi_ros extract_msdi_rld.launch
    ```

    - extract synced RGB+LWIR data

    ``` shell
    roslaunch msdi_ros extract_msdi_rl.launch
    ```

2. rosbag play

    ``` shell
    rosbag play -r 0.2 sequence_name.bag
    ```


### 3.2 extract and get dense depth images

If you want to be able to extract data in batch, you can use this script. This script will look for .bag files in the specified folder (**path_to_data_folder**) and extract the data to the target folder(**path_to_output_folder**) via the ROS launch (**ros_launchfile_name**). The script will also launch the script program with the calibration file (**path_to_calibfile**) in "rgbdutils" folder to obtain the standard camera view's depth images. Since there are missing data for some pixels in the directly obtained depth map, there is an option to use the Matlab program to complete the hole filling and obtain dense depth images. 


``` shell
./processBags.sh path_to_data_folder path_to_output_folder ros_launchfile_name path_to_calibfile 
```

e.g.,

``` shell
./processBags.sh /media/ubuntu/HDDData/multi-spectral-dataset/ \ 
    /media/ubuntu/nvmeData/multi-spectral-dataset/ \ 
    extract_msdi_rl.launch \
    rgbdUtils/calibfiles/extris_RGB2Kinect.yml
```

#### The depth processing pipline is shown below 

<div align="center">
<img src="images/depth-pipline.png" width="600" />
</div>

## Output Folder Directory

- **./times.txt**
    - Each line in the text file contains a single index timestamp for synced data.
    - timestamp (float) gives the number of seconds since the Unix epoch.
- **./color/*.png**
    - These color images are stored as 640x480 8-bit RGB images in PNG format.
- **./colorUndist/*.png**
    - These rectified color images are stored as 640x480 8-bit RGB images in PNG format.
- **./gray/*.png**
    - These gray images, converted from color images, are stored as 640x480 8-bit monochrome images in PNG format.
- **./thermalRaw/*.png**
    - These thermal images are stored as 640x480 16-bit monochrome images in PNG format.
    - In order to get temperature (Celsius) values in floating point format, the following conversion needs to be applied for each pixel:

``` c++
        float t = (float)data[i] / 10.f - 100.f;
```

- **./thermal/*.png**
    - These thermal images are stored as 640x480 8-bit monochrome images in PNG format.
    - The thermal images is obtain by transforming thermal raw using the affine parameters, shown in ab.txt, by the function:

``` latex
        I_{thermal} = e^{-a} (I_{thermal_raw} − b)
```

- **./ab.txt**
    - Each line in the text file contains a single affine transfer parameter pair.
    - The format of each line is 'timestamp a(float) b(float)'
- **./flag.txt**
    - Each line in the text file contains a single flag states.
    - The format of each line is 'timestamp flag_state'
    - the flag_state is defined as:

``` c++
        enum TFlagState {
            fsFlagOpen,
            fsFlagClose,
            fsFlagOpening,
            fsFlagClosing,
            fsError
        };
```

- **./depth/*.png**
    - These rectified depth images are stored as 640x480 16-bit monochrome images in PNG format.
    - Each depth image is inpainted with the corresponding image.
    - These depth images are scaled by a factor of 1000, i.e., a pixel value of 1000 in the depth image corresponds to a distance of 1 meter from the camera, 10000 to 10 meter distance, etc. A pixel value of 0 means missing value/no data.
- **./depthProj/*.png**
    - These rectified depth images are stored as 640x480 16-bit monochrome images in PNG format.
    - These depth images are scaled by a factor of 1000. A pixel value of 0 means missing value/no data.
- **./rgbdOver/*.png**
    - These color images are stored as 640x480 8-bit RGB images in PNG format.
    - Each image visualizes RGB and Depth alignment.
- **./kinectRGB/*.png**
    - These rectified color images are stored as 960x540 8-bit RGB images in PNG format.
- **./kinectDepth/*.png**
    - These rectified depth images are stored as 960x540 16-bit monochrome images in PNG format.
    - The depth images have been converted to the standard camera view.
    - These depth images are scaled by a factor of 1000. A pixel value of 0 means missing value/no data.
- **./kinectDepthCrop/*.png**
    - These rectified cropped depth images are stored as 640x480 16-bit monochrome images in PNG format.
    - These depth images are scaled by a factor of 1000. A pixel value of 0 means missing value/no data.
    - Each image is obtained by cropping the corresponding depth raw image in **./kinectDepth/** by the following code:

``` c++
        cv::Rect rect(160, 30, 640, 480);
        cv::Mat ROI(depth_raw, rect);
        cv::Mat croppedImage;
        ROI.copyTo(croppedImage);
```

- **./groundtruth.txt**
    - Each line in the text file contains a single pose.
    - The format of each line is 'timestamp tx ty tz qx qy qz qw'
    - tx ty tz (3 floats) give the position of the optical center of the standard camera with respect to the world origin as defined by the motion capture system.
    - qx qy qz qw (4 floats) provide the orientation of the optical center of the standard camera in the form of a unit quaternion with respect to the world origin as defined by the motion capture system.





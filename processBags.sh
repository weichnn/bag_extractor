#!/bin/bash

dir_folder=$(dirname $0)

data_root=$(realpath $1)
output_root=$(realpath $2)
launchfile_name=$3

if [ $# != 3 ]; then
  echo "False Usage!"
  echo " e.g.: $0 path_to_data_folder path_to_output_folder ros_launchfile_name"
  exit 1
fi

read -p "Do you want to impaint depth [yes/no]?: " impaintDepth

for filename in ${data_root}/*.bag ; do
    echo "processing ${filename}"
    name=$(basename "$filename" ".bag")
    if [ ! -f "$output_root/$name/groundtruth.txt" ]; then
        roslaunch msdi_ros ${launchfile_name} out_root:="$output_root/$name" &
        rosbag play -r 0.2 $filename
        if [ $? -eq 0 ]; then
            kill $(ps aux | grep 'roslaunch' | awk '{print $2}')
        fi
    fi
    if [ ! -d "$output_root/$name/depth" ]; then
        ${dir_folder}/rgbdUtils/generateDepth.sh ${dir_folder}/rgbdUtils/cppUtils/build/ ${dir_folder}/rgbdUtils/pyUtils/ ${dir_folder}/rgbdUtils/matlabUtils/toolbox_nyu_depth_v2/ ${dir_folder}/rgbdUtils/calibfiles/extris_RGB2Kinect.yml $output_root/$name $impaintDepth
    fi
done


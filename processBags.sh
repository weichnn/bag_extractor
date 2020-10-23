#!/bin/bash

dir_folder=$(dirname $0)

data_root=$(realpath $1)
output_root=$(realpath $2)
launchfile_name=$3
calib_file=$4

if [ $# != 4 ]; then
  echo "False Usage!"
  echo " e.g.: $0 path_to_data_folder path_to_output_folder ros_launchfile_name calib_file"
  exit 1
fi

read -p "Do you want to inpaint depth [yes/no]?: " inpaintDepth

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
        ${dir_folder}/rgbdUtils/generateDepth.sh ${dir_folder}/rgbdUtils/cppUtils/build/ ${dir_folder}/rgbdUtils/pyUtils/ ${dir_folder}/rgbdUtils/matlabUtils/toolbox_nyu_depth_v2/ $calib_file $output_root/$name $inpaintDepth
    fi
done


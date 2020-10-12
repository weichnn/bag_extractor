#!/bin/bash

data_root=""
output_root=""

for filename in ${data_root}/*.bag ; do
    echo "processing ${filename}"
    name=$(basename "$filename" ".bag")
    if [ ! -f "$output_root/$name/groundtruth.txt" ]; then
        roslaunch msdi_ros extract_msdi_rgblwir_dataset.launch out_root:="$output_root/$name" &
        rosbag play -r 0.2 $filename
        if [ $? -eq 0 ]; then
            kill $(ps aux | grep 'roslaunch' | awk '{print $2}')
        fi
    fi
done


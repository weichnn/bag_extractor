cpproot=$(realpath $1)
pyroot=$(realpath $2)
matlab_root=$(realpath $3)
calib_file=$(realpath $4)
root=$(realpath $5)
doInpaint=$6

core=$(($(nproc) - 3))

echo "------------------------"
echo "cpp root：${cpproot}"
echo "python root：${pyroot}"
echo "matlab root：${matlab_root}"
echo "color to kinect calib_file for proj depth：${calib_file}"
echo "data root：${root}"
echo "------------------------"

# matlab_root=/home/xlab/matlab/toolbox_nyu_depth_v2

if [ $# != 6 ]; then
  echo "False Usage!"
  echo " e.g.: $0 cpp_build_path python_code_path matlab_code_path calib_file data_root_path"
  exit 1
fi

if [ ! -d ${root}/tmp ]; then
  mkdir -p ${root}/tmp
fi

cd ${root}/kinectDepthCrop
ls *.png >../tmp/names.txt

colorNum=$(cat ../tmp/names.txt | wc -l)

echo "begin undistort rgb and project depth"
cd ${cpproot}
./undistort ${calib_file} ${root}/tmp/names.txt ${root}/color ${root}/colorUndist >${root}/tmp/undistort.txt &
./projDepth ${calib_file} ${root}/tmp/names.txt ${root}/kinectDepthCrop ${root}/depthProj >${root}/tmp/projDepth.txt
wait

if [[ $doInpaint == "yes" ]]; then
  echo "cd python path: ${pyroot}"
  cd ${pyroot}
  python creatCsv.py ${root}/colorUndist/ ${root}/depthProj/ ${root}/tmp/inpaint.csv --sort
  echo "associate depth and color, done"

  # matlab inpaint depth with color
  lineNum=$(grep -c "" ${root}/tmp/inpaint.txt)
  if [ ! -d ${root}/inpaintList ]; then
    mkdir -p ${root}/inpaintList
  fi
  split_line=$(awk 'BEGIN { rounded = sprintf("%.0f", '${lineNum}'/'${core}'); printf rounded }')
  echo "split ${lineNum} to ${core} file to ${root}/inpaintList, each file have line ${split_line}"
  split -d -l ${split_line} ${root}/tmp/inpaint.txt ${root}/inpaintList/inpaint
  echo "split done, begin matlab"

  dense_dir_rgb=${root}/depth/
  overlap_dir_rgb=${root}/rgbdOver/
  cd ${matlab_root}
  files=$(ls ${root}/inpaintList)
  for filename in $files; do
    filename=inpaintList/${filename}
    echo ${filename}
    matlab -nodisplay -r "root_dir='${root}/';list_file='${filename}';dense_dir='${dense_dir_rgb}';overlap_dir='${overlap_dir_rgb}'; fill_depth_batch" &
  done
  echo "matlab inpainting"
  wait
  echo "matlab done"
fi
# cat, shuf, sort, split, head, tail ...

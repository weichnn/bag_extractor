
echo "------------------------"
echo "build cpp utils"
echo "root：$0"
echo "------------------------"

dir_folder=$(dirname $0)

cd $dir_folder/cppUtils
mkdir -p build
cd build
cmake ..
make -j4
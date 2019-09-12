echo "Building ROS nodes"

cd Examples/ROS/ORB_Line_SLAM2
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j8

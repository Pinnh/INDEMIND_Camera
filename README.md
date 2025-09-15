# INDEMIND_Camera

A simpler way to read Indemind cameras' and imu's parameters and data (worked on ubuntu18.04 to ubuntu 22.04, ubuntu 24.04 is not test,may also work)

# Dependencies

opencv (Default not need, you can open in CMakeLists.txt by set(USE_OPENCV 1) and config opencv's path to show images. While ROS2 use default opencv.)

## Build and Run


mkdir build

cd build

cmake .. 

make

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:../driver/lib/x86-64 # may not need

sudo ./imsee_data_cap


# Build with ROS2


cd ros2/

sudo -s

source /opt/ros/humble/setup.sh #set to your ros2 path

colcon build --symlink-install

source install/setup.sh

ros2 run data_cap capture_image_imu


# INDEMIND_Camera

A simpler way to get Indemind camera and imu's data (worked on ubuntu18.04 to ubuntu 22.04, ubuntu 24.04 is not test,may also work)

# Dependencies

opencv4.x (default not need,when you want to show images,open in CMakeLists.txt by set(USE_OPENCV 1) and config it's path)

## Build and Run

`
mkdir build

cd build

cmake .. 

make

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:../driver/lib/x86-64

sudo ./imsee_data_cap
`

# Build with Ros2

`
cd ros2/

sudo -s

source /opt/ros/humble/setup.sh

colcon build --symlink-install

source install/setup.sh

ros2 run data_cap capture_image_imu
`

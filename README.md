# INDEMIND_Camera


## Build and Run

`
cd demo

mkdir build

cd build

cmake .. 

make

sudo ./data_cap
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

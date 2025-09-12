cmake_minimum_required(VERSION 3.8)
project(data_cap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

if (CMAKE_SYSTEM_NAME STREQUAL "Windows")
  message("WIN OS.")
  
elseif (CMAKE_SYSTEM_NAME STREQUAL "Darwin")
  message("MAC OS.")
elseif (CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message("LINUX OS.")
  
else()
  message(FATAL_ERROR "Unsupported OS.")
endif()

include_directories(include ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../../../driver/include )

add_executable(capture_image_imu src/capture_image_imu.cpp)
ament_target_dependencies(capture_image_imu rclcpp cv_bridge image_transport)
target_link_libraries(capture_image_imu ${CMAKE_CURRENT_SOURCE_DIR}/../../../driver/lib/x86-64/libusbdriver.so ${OpenCV_LIBRARIES})

install(
  TARGETS capture_image_imu
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


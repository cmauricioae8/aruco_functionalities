cmake_minimum_required(VERSION 3.5)
project(aruco_pose_estimation)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS aruco imgproc highgui)

set(dependencies "rclcpp" "sensor_msgs" "geometry_msgs"
"cv_bridge" "image_transport" "tf2" "tf2_ros" "tf2_geometry_msgs"
"std_msgs" "std_srvs")

## Define include directories
set(INCLUDE_DIRS include)

## IMPORTANT include dirs
include_directories(${ament_cmake_INCLUDE_DIRS} ${INCLUDE_DIRS})

add_executable(visual_pose_estimator src/visual_pose_estimator.cpp)
ament_target_dependencies(visual_pose_estimator ${dependencies})
target_link_libraries(visual_pose_estimator ${OpenCV_LIBRARIES})

install(TARGETS 
  visual_pose_estimator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()

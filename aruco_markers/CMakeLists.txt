cmake_minimum_required(VERSION 3.5)
project(aruco_markers)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(OpenCV 4 REQUIRED)

message("\nProject Dir: ${PROJECT_SOURCE_DIR}")
message("\nBinary Dir: ${CMAKE_BINARY_DIR}")


MACRO(add_example name)
  ADD_EXECUTABLE(${name} src/${name}.cpp)
  TARGET_LINK_LIBRARIES(${name} ${OpenCV_LIBRARIES})
ENDMACRO()


add_example(markers_generator)
add_example(camera_calibration)
add_example(aruco_detector)
add_example(pose_estimation_cube)


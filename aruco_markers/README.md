# ArUco Markers

This project provides a set of tools for working with ArUco markers using OpenCV. It includes marker generation, camera calibration, and real-time detection/pose estimation.

## Prerequisites

- **OpenCV 4.x** with the `aruco` module.
- **CMake** 3.5 or higher.
- **C++17** compiler.

## Building the Project

At 'aruco_markers' directory:
```bash
mkdir build
cd build
cmake ..
make
```

**NOTE:** To use a dummy camera calibration file, 'dummy_calibration_values.txt', use the following command (at build directory):

```bash
cp ../dummy_calibration_values.txt calibration_values.txt
```

To automatic build tests, run 'build_tests.sh' at 'aruco_markers' directory.


## Tools Overview

All tools support command-line arguments via `cv::CommandLineParser`. Use the `--help` flag with any executable to see available options.

### 1. Marker Generator (`markers_generator`)
Generates ArUco markers from a predefined dictionary and saves them as image files.
- **Key Parameters**: last_marker_id, dictionary type, etc.

**NOTE:** The markers MUST be printed on a white background, or at least a white border. 'dict_7x7_1000' folder contains a few examples of the markers.

### 2. Camera Calibration (`camera_calibration`)
Calculates the camera's intrinsic parameters and distortion coefficients using a chessboard pattern.
- **Key Parameters**:
  - `-s` : Square size (meters).
  - `-w` : Number of internal corners horizontally.
  - `-ny`: Number of internal corners vertically.
  - `-ci`: Camera index.
- **Output**: Saves `calibration_values.txt` in the build directory.

**NOTE:** The chessboard MUST be printed on a white background, or at least a white border.

### 3. ArUco Detector (`aruco_detector`)
Detects markers from the `DICT_7X7_1000` dictionary in real-time.
- **Key Parameters**:
  - `-s` : Marker size (meters).
  - `-ci`: Camera index.
- **Calibration**: Automatically attempts to load `calibration_values.txt` if available; otherwise uses default parameters.

### 4. Pose Estimation Cube (`pose_estimation_cube`)
Performs 3D pose estimation and draws a wireframe cube over the **largest detected marker** in the scene.
- **Key Parameters**:
  - `-l` : Marker length (meters).
  - `-ci`: Camera index.
- **Features**: Displays real-time XYZ position and orientation (Roll, Pitch, Yaw) for the largest marker.

### 5. Markers Cluster Spawner (`markers_cluster_gz`)
A ROS 2 package to spawn a grid of ArUco markers on a ceiling in **Gazebo Simulator (formerly Ignition)**.

- **Features**: 
  - Parameterizable grid (rows, cols, separation, height, size) via YAML.
  - Automatically assigns ArUcoTextures to each marker.
- **Usage (Gazebo Sim)**:
  1. Build the package:
     ```bash
     colcon build --packages-select markers_cluster_gz --base-paths .
     ```
  2. Start Gazebo Sim:
     ```bash
     ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="empty.sdf"
     ```
  3. Launch the spawner:
     ```bash
     source install/setup.bash
     ros2 launch markers_cluster_gz markers_cluster.launch.py
     ```

## License

Author: C. Mauricio Arteaga-Escamilla <cmauricioae8@gmail.com><br>
Source references are included in the individual source files.

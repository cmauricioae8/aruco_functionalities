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

## Tools Overview

All tools support command-line arguments via `cv::CommandLineParser`. Use the `--help` flag with any executable to see available options.

### 1. Marker Generator (`markers_generator`)
Generates ArUco markers from a predefined dictionary and saves them as image files.
- **Key Parameters**: last_marker_id, dictionary type, etc.

### 2. Camera Calibration (`camera_calibration`)
Calculates the camera's intrinsic parameters and distortion coefficients using a chessboard pattern.
- **Key Parameters**:
  - `-s` : Square size (meters).
  - `-w` : Number of internal corners horizontally.
  - `-ny`: Number of internal corners vertically.
  - `-ci`: Camera index.
- **Output**: Saves `calibration_values.txt` in the build directory.

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

## License

Author: C. Mauricio Arteaga-Escamilla <cmauricioae8@gmail.com><br>
Source references are included in the individual source files.

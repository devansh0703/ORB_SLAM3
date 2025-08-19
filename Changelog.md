# ORB-SLAM3
Details of changes between the different versions.

### V1.1, 18th August 2025 (Modernization Update)

- **Complete C++23 modernization**: Upgraded from C++11 to C++23 standard across entire codebase
- **OpenCV 4.6.0 compatibility**: Updated all OpenCV headers to modern API (opencv2/opencv.hpp)
- **Build system modernization**: CMake configurations updated for C++23 and latest dependencies
- **Third-party library updates**: DBoW2, g2o, and Sophus libraries updated for C++23 compatibility
- **Code compliance fixes**: Resolved namespace issues, mutex declarations, and C++17/C++23 strict type checking
- **Dependencies update**: All dependencies moved to latest stable versions (Eigen3 3.4.0, OpenCV 4.6.0)
- **Cross-platform testing**: Verified compatibility on Ubuntu 24.04 LTS with modern toolchain
- **Performance improvements**: Benefits from latest OpenCV optimizations and modern C++ features

All original functionality preserved with enhanced performance and modern development environment compatibility.

### V1.0, 22th December 2021

- OpenCV static matrices changed to Eigen matrices. The average code speed-up is 16% in tracking and 19% in mapping, w.r.t. times reported in the ORB-SLAM3 paper.

- New calibration file format, see file Calibration_Tutorial. Added options for stereo rectification and image resizing.

- Added load/save map functionalities.

- Added examples of live SLAM using Intel Realsense cameras.

- Fixed several bugs.

### V0.4: Beta version, 21st April 2021

- Changed OpenCV dynamic matrices to static matrices to speed up the code.

- Capability to measure running time of the system threads.

- Compatibility with OpenCV 4.0 (Requires at least OpenCV 3.0). 

- Fixed minor bugs.


### V0.3: Beta version, 4th Sep 2020

- RGB-D compatibility: the RGB-D examples have been adapted to the new version.

- Kitti and TUM dataset compatibility: these examples have been adapted to the new version.

- ROS compatibility: updated the old references in the code to work with this version.

- Config file parser: the YAML file contains the session configuration, a wrong parametrization may break the execution without any information to solve it. This version parses the file to read all the fields and give a proper answer if one of the fields have been wrongly deffined or does not exist.

- Fixed minor bugs.


### V0.2: Beta version, 7th Aug 2020
Initial release. It has these capabilities:

- Multiple-Map capabilities: it is able to handle multiple maps in the same session and merge them when a common area is detected with a seamless fussion.

- Inertial sensor: the IMU initialization takes 2 seconds to achieve a scale error less than 5\% and it is reffined in the next 10 seconds until it is around 1\%. Inertial measures are integrated at frame rate to estimate the scale, gravity and velocity in order to improve the visual features detection and make the system robust to temporal occlusions.

- Fisheye cameras: cameras with wide-angle and fisheye lenses are now fully supported in monocular and stereo. 



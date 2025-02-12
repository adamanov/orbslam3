# ORB_SLAM3_ROS2
This repository is the modified package of the [original repository](https://github.com/zang09/ORB_SLAM3_ROS2). \
The issue I had is 
* rclcpp node does not provide anything (tf2, tracked image, pose topics etc)
* stereo-inertial node did not work because of QOS setting in IMU subscriber.
* SLAM pose was optical frame pose expressed in OpenCV frame, but I needed camera frame pose in ROS FLU coordinate.

Standalone C++ ORB-SLAM3 repository is [here](https://github.com/jnskkmhr/ORB-SLAM3-STEREO-FIXED.git). \
It is modified for this repository.

---

## Prerequisites
Current repository supports:
  - Ubuntu 20.04
  - ROS2 foxy
  - OpenCV 4.2.0

In the future, I will also test with Ubuntu22.04 (OpenCV 4.5.4).

## Installation
**Please try native installation only if you know what you are doing!!** \
First, clone the repository. 
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone git@github.com:jnskkmhr/orbslam3.git
```

### Build docker image
We use docker for the simplicity. \
You can build docker image via
```bash
cd orbslam3
./docker/build_image.sh
```

### Build ros2 package

To build orb-slam3 ros2 package, you need to start container and build the package inside the container. \
To do so, you first run the following command and run colcon build inside docker shell.
```bash
# the following shell command let you enter docker container and cd to /home/ros2_ws
./docker/run_container.sh

# inside docker container, build orbslam3 ros2 package
colcon build --cmake-args -DCMAKE_CXX_FLAGS="-w" --symlink-install --packages-select orbslam3
```
Be aware that you only need to build package once since the entire ros2_ws directory is mounted to docker container.

## How to use
1. Start container and source the workspace.

```bash
# the following shell command let you enter docker container and cd to /home/ros2_ws
./docker/run_container.sh
# then inside the container, source ros2 workspace
source install/setup.bash
```

2. Run orbslam mode, which you want.  

Currently, I modified stereo and stereo-inertial node. \
First, start realsense node. \
For example, to run realsense camera with stereo and imu streams, 
```bash
ros2 launch realsense2_camera rs_launch.py enable_infra1:=true enable_infra2:=true enable_accel:=true enable_gyro:=true unite_imu_method:=2 infra_width:=640 infra_height:=480 camera_name:=d455 camera_namespace:=d455
```

Stereo node:
```bash
ros2 launch orbslam3 stereo_d455.launch.yaml
```

Stereo-Inertial node:
```bash
ros2 launch orbslam3 stereo_inertial_d455.launch.yaml
```

## Topics

### Stereo
Subscriber:
* `/camera/left`: left image topic
* `/camera/right`: right image topic

Publisher:
* `/camera_pose`: left camera frame pose in ROS FLU map (first camera frame) coordinate 
* `/tracking_image`: tracked left image. 
* `/tf`: `map` to `odom_frame` transform (need external odometry source)

### Stereo-inertial
Subscriber:
* `/camera/left`: left image topic
* `/camera/right`: right image topic
* `/imu`: imu topic

Publisher:
* `/camera_pose`: left camera frame pose in ROS FLU map (first camera frame) coordinate 
* `/tracking_image`: tracked left image. 
* `/tf`: `map` to `odom_frame` transform (need external odometry source)

Note that the orbslam3 node publishes map to odom tf2. \
But, you can also publish map to camera instead by commenting out the [following](https://github.com/jnskkmhr/orbslam3/blob/28a55556bb3be2e3065b1bb4eedf9f99227c5c51/src/stereo/stereo-slam-node.cpp#L142-L154) and turning on the [following](https://github.com/jnskkmhr/orbslam3/blob/28a55556bb3be2e3065b1bb4eedf9f99227c5c51/src/stereo/stereo-slam-node.cpp#L157). 
The same goes to stereo-inertial node. 


### WIP
Currently, stereo-inertial node does not work properly. \
I see IMU initialization takes a lot of time, and the pose sometimes jumps. \
This might be because of wrong calibration configuration. \
Please send PR if you know how to solve issues.

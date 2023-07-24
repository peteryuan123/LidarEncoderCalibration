# LidarEncoderCalibration


This project aims to calibrate the 4DOF extrinsic between a lidar and the encoder. The extrinsic is used in Lidar SLAM motion compensation.
This setting is only for testing.

<img src="https://github.com/peteryuan123/LidarEncoderCalibration/assets/33962726/fa2a05d3-eb61-40c6-84c5-aab035bf1273" width="400px">
<img src="https://github.com/peteryuan123/LidarEncoderCalibration/assets/33962726/8fc431b9-6419-4f3c-bacf-cf4827d6c99d" width="375px">

### Prerequisites 

```
PCL
Ceres
ROS(noetic)
```

### Usage example

OS X & Linux:
ubuntu 20.04

```sh
catkin_make
source devel/setup.sh
roslaunch rotationCalibration.launch
```

### DEMO
The reconstruction using the calibrated extrinsic.
![image](https://github.com/peteryuan123/LidarEncoderCalibration/assets/33962726/83629a83-4608-4f5d-a602-cbd0e6d92a97)


# openpose_ros2
English | [日本語](./README_ja.md)

A ROS2 package that call the OpenPose from ROS2.

## Required
* Python 3.6
* ROS2 dashing
* cv_bridge
* [OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose)
    * Python Wrapper

## Checked environment
* Python 3.6
* Ubuntu 18.04 LTS
* ROS2 dashing
* OpenPose v1.7.0

## Installation
1. Install [OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose). This package requires OpenPose Python Wrapper.
1. Clone this repository to your ros2 workspace (substantially `~/ros2_ws/src` ).
1. Change directory ros2 workspace root (substantially `~/ros2_ws` ), and build package with `colcon build --base-paths src/openpose_ros2` .
1. Update packages with `. install/setup.bash` .

## Run node with params.yml (**RECOMMEND**)
1. Change directory ros2 workspace root.
1. Run `cp ./src/openpose_ros2/openpose_ros2/params.yml.sample ./src/openpose_ros2/openpose_ros2/params.yml` .
1. Edit `./src/openpose_ros2/openpose_ros2/params.yml`. Please see "ROS2 parameters".
1. Run `ros2 run openpose_ros2 openpose_ros2 __params:=src/openpose_ros2/openpose_ros2/params.yml` .

## Run node with launch command
1. Change directory ros2 workspace root.
1. Edit `./src/openpose_ros2/openpose_ros2/launch/openpose_ros2.py` . Please see "ROS2 parameters".
1. Build package with `colcon build --base-paths src/openpose_ros2` and `. install/setup.bash` .
1. Run `ros2 launch openpose_ros2 openpose_ros2.py` .

## Nodes
* /openpose_node

## Topics
* /openpose/pose_key_points \[openpose_ros2_msgs/msg/PoseKeyPointsList\]
* /openpose/preview \[sensor_msgs/msg/Image\] (debug mode only)
* /openpose/preview/compressed \[sensor_msgs/msg/CompressedImage\] (debug mode only)

## ROS2 parameters
* is_debug_mode : If true, run debug mode. 
* openpose_root : The path of openpose root dir.
* is_image_compressed : If true, input images expect CompressedImage. Otherwise, input images expect Image.
* image_node : The string of input image node.

## License
MIT License

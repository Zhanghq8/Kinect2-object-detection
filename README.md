# Kinect2-object-detection   
## Maintainer
- [Hanqing Zhang], <<hzhang8@wpi.edu>>, WPI   
*Note:* This project was based on [Thiemo Wiedemeyer]'s work:(https://github.com/code-iai/iai_kinect2).   
## Read this first
- Follow the guideline here: (https://github.com/code-iai/iai_kinect2) to setup your Kinect2 device in your ROS workspace.
- Substitute the [Kinect2_viewer] folder with this new one. 

## Dependencies

- ROS Kinetic
- OpenCV
- PCL
- Ubuntu 16.04

## Description   
1.Object detection by color filtering  
- [color_test.cpp] this node is for getting the correct HSV value for specfic color of the object.
- [pose_color.cpp] this node is for detecting and getting the 3d pose of the object.    

2.Object detection by cloud filtering
- [pose_cloud.cpp] this node is for detecting and getting the 3d pose of the object.

## Run
1.Activate your KInect2 device by running `roslaunch kinect2_bridge kinect2_bridge.launch`.   
2.For [Object detection by color filtering]:
- First run `rosrun kinect2_viewer color_test` to tune the HSV value to get the desired result;
- Substitute the HSV value in [pose_color.cpp] with the new one;
- Do `catkin_make` in your workspace and then run `rosrun kinect2_viewer pose_color`.   
2.For [Object detection by loud filtering]:
- First substitute the depth range for your goal;
- Do `catkin_make` in your workspace and then run `rosrun kinect2_viewer pose_cloud`.

## Screenshots

Here are some screenshots of the results for this project:    
- For [Object detection by color filtering]:
- For [Object detection by cloud filtering]:   
![pose_cloud image](https://github.com/Zhanghq8/Kinect2-object-detection/blob/master/pose_cloud.png)

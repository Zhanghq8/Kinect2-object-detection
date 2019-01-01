# Kinect2-object-detection   
## Maintainer
[Hanqning Zhang] <<hzhang8@wpi.edu>>, WPI   
*Note:* This project was based on [Thiemo Wiedemeyer]'s work:(https://github.com/code-iai/iai_kinect2).   

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

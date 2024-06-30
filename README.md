# ROS2 package to publish webcam/usbcam/ip_address stream as ROS2 topics

Hello all, I have made this package to send video footage from a webcam, USB camera, or an ip_address onto a ros2 topic, which you can use for openCV in your ros2 projects. The main reason to create this package was to get video streaming from my phone on to an ip address and use that stream as ros2 topic. After searching for a while i could find 2 repositories which are really good but for ip_address streaming i could not find so i created this package. Its very simple and has 3 nodes.

## Similar Libraries
1. Opencv_cam - https://github.com/clydemcqueen/opencv_cam
2. usb cam  - https://github.com/ros-drivers/usb_cam 

## Table of Contents

- [Install](#install)
- [Usage](#usage)

## Install

Go to your work_space/src folder, and create one if you don't have.

To create a workspace and src folder
```sh
$ mkdir ros2_ws/src
$ cd ros2_ws/src
```

clone the repo in the src folder
```sh
$ git clone https://github.com/pratikPhadte/my_camera.git
```
Go back to the workspace folder (ros2_ws/)
```sh
$ cd ..
```
Once you are in your workspace, we can start building the repo
```sh
$ colcon build --packages-select my_camera
```

IF its showing you an error while building in ROS2 humble and Ubuntu 22.04 version, try building it in sequential mode. if your "my_camera" is built successfully then please skip this step.
```sh
$ colcon build --executor sequential --packages-select my_camera
```
IT's DONE! 

## Usage

Go to your work_space/ folder.
```sh
$ cd ros2_ws
```

Always remember to source your workspace :)
```sh
$ source install/local_setup.bash
```

Run the ROS2 webam_node, this node will publish the webcam feed onto a ros2 topic.
```sh
$ ros2 run my_camera webcam_node
```
![image](https://github.com/pratikPhadte/my_camera/assets/55589461/ca8f4295-06de-40f4-8a98-1249b82f44bd)

If you want to see usb webcam footage or RC/fpv camera receiver footage, use below a node, instead of 0( 0 is a deafult computer webcam number), you can put 1,2,3.. to check your connected usb cam.
```sh
$ ros2 run my_camera usbcam_node --cam 0
```
If you want to see an ip_address feed, use below command, note that you have to add the PORT number at the end :)
```sh
$ ros2 run my_camera ip_stream_node --ip 192.168.0.180:8080 
```
![image](https://github.com/pratikPhadte/my_camera/assets/55589461/8ea8d5b6-e6e3-45c3-9752-37875b23f5a0)

In another terminal, check the ros2 topics
```sh
$ ros2 topic list
```
![image](https://github.com/pratikPhadte/my_camera/assets/55589461/01d14425-d210-4548-b6c5-d96d0d0c6bd2)

To view your footage, enter below command in another terminal.
```sh
$ ros2 run rqt_image_view rqt_image_view
```
![image](https://github.com/pratikPhadte/my_camera/assets/55589461/efc694f6-4a7c-44c9-bd92-6c5256b1ca4f)



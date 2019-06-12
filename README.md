# Stereo images depth estimation

## Overview

In the reposotory there are two ROS packages:

- <b>stereo_depth_estimation</b> - ROS package that estimates depth being given a pair of stereo images using a neural network and publishes the result (estimated depth) as an image. All the main functionalities are packed in here.
- <b>stereo_img_publisher</b> - publishes stereo images on desired topic to run a stereo_depth_estimation node. Optional, used for debugging and demo purposes.
- <b>stereo_img_saver</b> - saves output images of the network. Optional, used for debugging and demo purposes.

In order the network to work better, input pair of images should be rectified (e.g. with stereo_image_proc). Furthemore, coloured (RGB8) and the cameras ought be calibrated (e.g. with kalibr).

## Quick depth estimation demo
<b>IMPORTANT NOTE</b> - remember to source the setup.bash file for given catkin workspace each time you open a new terminal!
- Go to the ```{catkin_workspace}/src``` directory
- Clone the repo with ```git clone https://github.com/michalMalujdy/ROS-image-depth-estimation.git```
- Go to the ```{catkin_workspace}``` driectory
- Run ```catkin_make```
- In a terminal run ```roscore``` to run ROS environment
- In another terminal run ```rosrun stereo_depth_estimation main.py -show``` to start the neural network to estimate the depth
- In another terminal run ```rosrun stereo_img_publisher main.py``` to publish a pair of stereo images that the stereo_depth_estimation node subscribes to

## Usage of the stereo_depth_estimation package
It is recommended to use ```stereo_depth_estimation/main.launch``` in order to run the package. Configurable parameters are placed there:
- ```input_img_left``` - the left input image topic that the node will subscribe to. Please set the value with respect to your camera left image output topic.
- ```input_img_right``` - the right input image topic that the node will subscribe to. Please set the value with respect to your camera right image output topic.
- ```img_out``` - the output depth image topic that the node will publish to. 

Steps to run the node:
- Edit parameters in the ```stereo_depth_estimation/main.launch``` as mentioned above
- Source the environment
- Run ```roslaunch stereo_depth_estimation main.launch```


## ROS topics

The node subscribes on two input topics for left and right image and publishes the result depth image on the output topic.

As mentioned previously, the three topics are configurable, however the default values are as follows:

- ```/stereo_depth_estimation/input/left```
- ```/stereo_depth_estimation/input/right```
- ```/stereo_depth_estimation/output```

## Structure

### stereo_depth_estimation

The ```src/data``` directory holds the ```src/data/000000.txt``` configuration file and the network model info in ```src/data/model-inference-513x257-0.meta``` and ```src/data/model-inference-513x257-0```

In the ```src/scripts/network.py``` file lies the implementation of the core neural network that uses tensorflow.

In the ```src/scripts/main.py``` file lies ROS node wrapper for the network. Subscriptions for images are placed there, packing and unpacking image messages, callback for the subscriptions.

### stereo_img_publisher

Directories ```img/left``` and ```img/right``` holds the test images that are going to be published once the node is started. They can be swaped for a different ones but names need to be either preserved or updated in the ```src/main.py``` file.

The file ```src/main.py``` holds the whole source code of the package.

# Calibration
In order the network to perform well, stereo cameras need to be calibrated. In the project for calibration purpouses the [kalibr](https://github.com/ethz-asl/kalibr/wiki/installation) package was used. The installation and usage process of the kalibr package in the project was done as follows at the [wiki page](https://github.com/ethz-asl/kalibr/wiki/installation). As result the calibration file is output from the kalibr package used in the further process of the input images processing (rectifying).

# Input image processing
The network was trained on rectified images and in order to perform well it needs rectified input images. In order to do so, in the project [stereo_image_proc](http://wiki.ros.org/stereo_image_proc) was used. In the initialization it needs calibration data acquired from the previous section (using kalibr). The quickstart, usage and output topics are described in the [wiki](http://wiki.ros.org/stereo_image_proc).

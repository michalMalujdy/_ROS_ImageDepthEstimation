# Stereo images depth estimation

## Overview

In the reposotory there are two ROS packages:

- <b>stereo_depth_estimation</b> - ROS package that estimates depth being given a pair of stereo images using a neural network and publishes the result (estimated depth) as an image.
- <b>stereo_img_publisher</b> - publishes stereo images on desired topic to run a stereo_depth_estimation node.

## Quick depth estimation demo
<b>IMPORTANT NOTE</b> - remember to source the setup.bash file for given catkin workspace each time you open a new terminal!
- Go to the ```{catkin_workspace}/src``` directory
- Clone the repo with ```git clone https://github.com/michalMalujdy/ROS-image-depth-estimation.git```
- Go to the ```{catkin_workspace}``` driectory
- Run ```catkin_make```
- In a terminal run ```roscore``` to run ROS environment
- In another terminal run ```rosrun stereo_depth_estimation main.py -show``` to start the neural network to estimate the depth
- In another terminal run ```rosrun stereo_img_publisher main.py``` to publish a pair of stereo images that the stereo_depth_estimation node subscribes to


## ROS topics

A stereo_depth_estimation node subscribes on two topics to receive desired pair of stereo images:

- stereo_depth_estimation/img/left
- stereo_depth_estimation/img/right

Once images are published in those topics (within time margin of 10ms between left and right channel) the stereo_depth_estimation runs the depth estimation.

## Structure

### stereo_depth_estimation

The ```src/data``` directory holds the ```src/data/000000.txt``` configuration file and the network model info in ```src/data/model-inference-513x257-0.meta``` and ```src/data/model-inference-513x257-0```

In the ```src/scripts/network.py``` file lies the implementation of the core neural network that uses tensorflow.

In the ```src/scripts/main.py``` file lies ROS node wrapper for the network. Subscriptions for images are placed there, packing and unpacking image messages, callback for the subscriptions.

### stereo_img_publisher

Directories ```img/left``` and ```img/right``` holds the test images that are going to be published once the node is started. They can be swaped for a different ones but names need to be either preserved or updated in the ```src/main.py``` file.

The file ```src/main.py``` holds the whole source code of the package.
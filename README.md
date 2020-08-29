# End-to-End Self Driving Car :oncoming_automobile:
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

---

Udacity Self-Driving Car Engineer Nanodegree

Capstone Project

---

## Introduction

This is the Capstone project for the Udacity Self-Driving Car Nanodegree. We developed software to guide a real self-driving car around a test track. Using the Robot Operating System (ROS), we created nodes for traffic light detection and classification, trajectory planning, and control.

![Result](https://github.com/Ansheel9/End-to-End-Self-Driving-Car/blob/master/extra/sd1.gif)

The project proceeded in two stages. First, basic elements of the system were developed to run on Udacity’s simulator (Figure 1). The simulator is a virtual reality 3D environment with three lanes of traffic and eight lights. There are no other cars in the environment. In order to pass the first test, the simulated vehicle must stop “safely”, that is smoothly and before the white line, at each red light, decide to stop or continue through each yellow light, and continue through each green light at an appropriate speed.

![Result](https://github.com/Ansheel9/End-to-End-Self-Driving-Car/blob/master/extra/sd2.jpg)

Once the software system passes the first test on the simulator, it is transferred to Udacity’s Carla (Figure 2) to run on a real-world (albeit greatly simplified) track. Of note, the traffic lights encountered on the test track are substantially different than the traffic lights encountered in the simulator. Moreover, conditions on the test track include glare and poor contrast among other challenges.

## Architecture

ROS is an open source software framework developed initially by Willow Garage (http://www.willowgarage.com) and is ideally suited for complex robot operations that often involve multiple systems running independently. For example, the multiple modules in Carla include: 1) a camera that produces raw images, 2) the object detection and classification system which consumes the raw images and produces the state of the light (i.e., green, yellow, red), 3) a decision-making system that decides whether to stop at a light or continue through it, and 4) the vehicle control system that is responsible for steering, accelerating and decelerating the car, just to name a few. Indeed, most robots and autonomous vehicles are may contain dozens of interacting modules. Writing a single program that integrates all of the modules in a self-driving car would be very challenging, to say the least.

![Result](https://github.com/Ansheel9/End-to-End-Self-Driving-Car/blob/master/extra/sd3.png)

ROS allows these systems to run independently while, at the same time, exchanging messages. The main mechanism for communicating between modules is through a “publisher-subscription” (commonly referred to as a “pub-sub” model). In certain cases, asynchronous communications, in which one subsystem can interrupt and block the processing of another system, are required. ROS supports both message passing protocols.

Another important benefit of using ROS is the large number of device drivers that are included with the distribution. These device drivers support many of the most popular sensors (cameras, LIDAR, RADAR, IMU) and actuators (motors and controls). The easy integration of the supported devices means that developers are able to focus more effort on integration.

ROS has several tools that aid in the development of an integrated system. The system developer can “peer into” the messages being exchanged and store these in a “bag” file, which can can be replayed to the system. Replaying past data ensures that conditions can be replicated during failure analysis.

***Sensing.*** The sensing subsystem is comprised of several cameras, an inertial measurement unit (IMU), and RADAR and LIDAR sensors. The RADAR is forward facing and only senses the distance to objects in front of the car.  The LIDAR system has a 360-degree view that provides distance to objects in all directions. The GPS provides an estimate of position in global coordinates with 1-2 meters resolution. The IMU estimates displacement of the car in the x (forward-backward), y (left-right) and z (up-down) directions, along with the angular displacement.  

***Perception.***  As in neural and cognitive science, perception differs from sensation in that perception takes the raw input from the sensor and elaborates it into meaningful information.  Meaningful information for a car include traffic light detection and classification (as included in this project), obstacle detection, traffic sign detection, detection of other cars, and, pedestrians.  A substantial part of the effort in the Capstone project involved perceiving the state of the traffic lights.  

***Planning.***  Self-driving cars (and the drivers they will replace) always need to think ahead.  The planning subsystem used in this project was built on the concept of ***waypoints***—a series of coordinates that are updated as the car moves through the environment.  For example, a lane change to the left might be comprised of 20-40 equidistant waypoints in the form of a smooth *s* shape over a 30 meter span.  Note: there is no requirement that the waypoints are equidistant, extend 30 meter, nor that there are 20-40 waypoints that describe the trajectory. There are many possibilities.  In dense traffic, the parameters may vary substantially from the parameters in sparse traffic. A real-world planning subsystem would be adaptive to the situation.

***Control.***  Finally, after planning comes execution of the commands that control the car.  As in human driving, there are a limited number of controls that, in combination, create a complex series of behaviors.  Carla is equipped with a drive-by-wire controller to control __1)__ acceleration, __2)__ braking and __3)__ steering.  Carla uses a proportional-integral-derivative (PID) controller.  

## Object Detection

In order to react correctly to the traffic lights, the software system must achieve: __1) Detection.__ Identify the traffic light housing with a bounding box and __2) Classification.__ Look within the bounding box to determine the state of the light (green, yellow, red). This can be done by taking advantage of ***transfer learning*** in which the object detection network is pretrained on a huge image datasets such as the [COCO Dataset](http://cocodataset.org/). [TensorFlow Detection Model Zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md) offers a number of collection of detection models pre-trained on massive datasets. The pretrained network selected for use is the `ssd_mobilenet_v2_coco`.

![Result](https://github.com/Ansheel9/End-to-End-Self-Driving-Car/blob/master/extra/sd4.png)

Since the automotive hardware is closer to mobile or embedded devices than cloud GPUs, the MobileNet neural network designed for running very efficiently (high FPS, low memory footprint) on mobile devices, was integrated as the base network. The MobileNet can reduce the size of cummulative parameters and therefore the computation required on automotive/ mobile hardwares with limited resources ([Andrew et al. 2017](https://arxiv.org/abs/1704.04861>)).


Note that after identify the traffic lights, we will need to run another classification network to detect the state of the light.

**We can bypass the need to run separate networks to identify traffic light and classify the state of the light by making our own custom object detector that immediately detects the state of the traffic lights instead. By doing so, we will only need to run a single network which saves us computational power and it will run faster than having two separate network.**

### Notes about traffic light detection and classification
* Pre processing: image is resized to 300x300 pixel in RGB format
* Traffic light is class 10, so we use detections only with this class.
* Selected the highest probability traffic light.
* Crop a small image based on bounding box coordinates (with small padding).
* Color classification is calculated based on the number of high intensity red and green pixels on the cropped image.
___

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

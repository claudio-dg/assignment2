Experimental Robotics Lab Second assignment
================================
This repository contains the work for the second assignment of Experimental Robotics Laboratory

#### Author: Claudio Del Gaizo, S4696649

#### mail: cdg9914@gmail.com


Table of contents
----------------------

* [Introduction](#introduction)
* [Dependencies and Setup](#dependencies-and-setup)
* [My Robot](#my-robot)
* [Project structure](#project-structure)
* [Software Components and code description](#software-components-and-code-description)
* [Behaviuor Presentation](#behaviuor-presentation)
* [Limitations and Possible Improvements](#limitations-and-possible-improvements)


## Introduction

The goal of this assignment is to develop a software architecture to control a robot, which is deployed in a indoor environment for surveillance purposes. The robot’s objective is to visit the different locations and stay there for some times.

The 2D environment has to be produced making use of Armor_api to create an ontology, and should resemble the following map, made of 4 rooms and 3 corridors:

<p><p align="center">
<img src="https://github.com/claudio-dg/assignment_1/blob/main/images/map.png?raw=true" width="400" />
<p>
	
	
<!--  
-->
Within this environment, the robot should:

1. start in the E location and wait until it receives the information to build the 
```topological map```, i.e. the relations between C1, C2, R1, R2, R3 locations and the doors D1...D6.
2. move in a new location, and should wait for some time before visiting another location. This 
behavior is repeated in a infinite loop. When robot’s battery is not low, it should move among locations with this ```policy```:
- It should mainly stay on corridors,
- If a reachable room has not been visited for some time, it becomes ```URGENT``` and the robot should visit it.
3. When the robot’s ```battery is low```, it should go in the E location, and wait for some time before starting again with the above behavior

<!-- To define the behaviour of the robot we have to create a Finite States Machine using SMACH libraries -->





##  Dependencies and Setup

In order to run correctly the project of this repository, some important dependencies have to be taken into account, therefore please make sure to have the following packages already installed in your ```ros_workspace```:
- [arch_skeleton](https://github.com/buoncubi/arch_skeleton): from which I retrieved useful [actions](https://github.com/buoncubi/arch_skeleton/tree/main/action), [messages](https://github.com/buoncubi/arch_skeleton/tree/main/msg) [services](https://github.com/buoncubi/arch_skeleton/tree/main/srv) and [utilities](https://github.com/buoncubi/arch_skeleton/tree/main/utilities/arch_skeleton)	to implement the planner and controller of this repository. Please note that credits for the planner and controller also go this repository, since they have been implemented starting from the example there shown and slightly adapting them to my needs.
- [topological_map](https://github.com/buoncubi/topological_map): from which I retrieved the [topological_map.owl](https://github.com/buoncubi/topological_map/blob/main/topological_map.owl), to use it as starting point for creating my ```ontology``` of the environment for the assignment, which will be created in run time and saved as a .owl file in the ```topological_map``` folder.
- [aRMOR](https://github.com/EmaroLab/armor): which is necessary to use an OWL ```ontology``` and the related ```reasoner``` within ROS. Particularly useful is the [armor_py_api](https://github.com/EmaroLab/armor_py_api), which simplifies the calls to aRMOR, but can only be used from a python-based ROS node, and it allows to interact with ontologies with three types of operations: manipulations, queries, and ontology management.

In the end, the Finite States Machine here implemented is based on [SMACH](http://wiki.ros.org/smach) libraries.
	
When all these are correctly installed, to try this repository it is necessary to: clone it in your ROS workspace: 

```bash
$ git clone https://github.com/claudio-dg/assignment_1.git
```

then type the following commands in the terminal to make sure to launch the rosmaster as well as the aRMOR service:

```bash
$ Roscore &
$ rosrun armor execute it.emarolab.armor.ARMORMainService

```
Please note that after building the package for the first time it may be required to go to the project directory and run the following command to correctly use aRMOR

```bash
$ ./gradlew deployApp
```
	
After that you can type the following command in the terminal to simultaneously launch all the necessary nodes through the [launchFile](https://github.com/claudio-dg/assignment_1/tree/main/launch):

```bash
$ roslaunch assignmnent_1 start_simulation.launch
```
	
## My Robot
	image and robot description
	
	
	
## Project structure

The project is based on the ROS scheme that is shown in the following ```rqt_graph```:

<p align="center">
<img src="https://github.com/claudio-dg/assignment_1/blob/main/images/assignment_1_rosgraph.png?raw=true" width="850" />
<p>
 
This repository contains a ROS package called ```"assignment_1"``` that includes the following resources:

- [CMakeList.txt](https://github.com/claudio-dg/assignment_1/blob/main/CMakeLists.txt): File to configure this package.
- [package.xml](https://github.com/claudio-dg/assignment_1/blob/main/package.xml): File to configure this package.
- [Docs/](https://github.com/claudio-dg/assignment_1/tree/main/Docs): folder containing ```Doxygen documentation``` of the package
- [images/](https://github.com/claudio-dg/assignment_1/tree/main/images): folder containing images and graphs used within this [README](https://github.com/claudio-dg/assignment_1/blob/main/README.md).
- [scripts/](https://github.com/claudio-dg/assignment_1/tree/main/scripts): It contains the implementation of each software components produced for this project.
	
	* [FSM.py](https://github.com/claudio-dg/assignment_1/blob/main/scripts/FSM.py): contains the implementation of the SMACH Finite States Machine
	
	* [controller.py](https://github.com/claudio-dg/assignment_1/blob/main/scripts/controller.py): It is a dummy implementation of a motion controller. (Credits to [arch_skeleton](https://github.com/buoncubi/arch_skeleton))
	
	* [helper.py](https://github.com/claudio-dg/assignment_1/blob/main/scripts/helper.py): It contains the implementation of two helper classes for ROS actions and for interfacing the Ontology through aRMOR_api
	
	* [planner.py](https://github.com/claudio-dg/assignment_1/blob/main/scripts/planner.py): It is a dummy implementation of a motion planner. (Credits to [arch_skeleton](https://github.com/buoncubi/arch_skeleton))
	
	* [robot_state.py](https://github.com/claudio-dg/assignment_1/blob/main/scripts/robot_state.py): It implements the robot state including: current position, and battery level.
- [launch/](https://github.com/claudio-dg/assignment_1/tree/main/launch): It contains the launch file to start the simulation
	
	* [start_simulation.launch](https://github.com/claudio-dg/assignment_1/blob/main/launch/start_simulation.launch): launch file of the project

	

 
## Software Components and code description
	





	
 

## Behaviuor Presentation
 
Video aruco detection
****
DEMO VIDEO
Here you can find a short video (by clicking on the image) showing the resulting simulation of my project. Please notice that most of the video has a x10 speed up and also that some numbers will Pop-Up in the screen to highlight the crucial moments of the simulation: for a better understanding such numbers are asscoiated to a text description of the behaviour that you can find here just below the video.

[![Click to see Simulation Demo](http://img.youtube.com/vi/YOUTUBE_VIDEO_ID_HERE/0.jpg)](http://www.youtube.com/watch?v=YOUTUBE_VIDEO_ID_HERE "Video Title")

**1.** After having detetcted all markers, the robot plans to move to C2 and starts its motion, waiting for the goal to be reached or for the battery to get low.
	
**2.** The robot has reached the goal, since it is a corridor it does not apply survey algorithm, but plans to move to R3 Room and starts doing it.
	
**3.** Once R3 is reached the robot rotates the camera of 360 degrees, and plans to move back to the previous corridor.
	
**4.** Before reaching the exact coordinates of C2 the batery gets low: robot enters in Recharging state of the Finite state machine, and since he hasn't reached yet the corridor it believes to be still in R3; for this reason it plans again to reach C2, and once it reaches its exact coordinates it plans to reach the recharging station and moves towards it.
	
**5.** In the end the robot reaches the Recharging station and starts recharging the battery. It waits until it gets full charge, and then restarts its routine by moving towards a corridor.

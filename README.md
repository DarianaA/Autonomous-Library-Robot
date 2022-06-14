# Library-Autonomous-Robot

This project aims to program an autonomous robot, capable of recognising objects and manipulating them. The solution incorporates three main modules: perception, navigation with path planning, and manipulation.

The detection and selection of the object to be grasped is performed using a convolutional neural network architecture (YOLO) and OpenCV library. For the manipulation part, the robot first generates a number of grasps based on the point clouds of the objects and then select the best grasping approach. Navigation in the environment is implemented using the move_base package.

The project has been developed using two workspaces: tiago_ws containing the configuration of the TIAGo robot and basic functionalities and packages (following the installation tutorial from the TIAGo ROS wiki page), and this workspace, which contains the actual implementation of the behaviour of the robot, as well as other requirements and dependencies needed, such as the container definition.

This workspace is a modified version of the workspace configured by the KCL-Leeds RoboCup team. It contains the main functionalities implemented to accomplish the final behaviour of the assistant robot, as well as other dependencies and requirements for the general success of the project.

The main functionalities are: object detection using YOLO, a program for taking pictures of objects (pictures that are used when training YOLO), grasps generation, and manipulation. All of the robot's actions are subsequently executed using a state machine.

Because the implemented solutions require 2 different configured workspaces (one configured with python3 for YOLO and grasping, and one configured with python2 for navigation), a singularity container represented the best way to package up the multiple versions of the same software into a single file, allowing the system to run commands on both workspaces at the same time.

A singularity container is used to encapsulate all required software and dependencies in a workflow. More than that, it provides integration over isolation by default, meaning that it easily makes use of GPUs, high speed networks and parallel file systems. Because running YOLO through the CPU is very slow, the run-time performance can be increased by using a CUDA enabled GPU.

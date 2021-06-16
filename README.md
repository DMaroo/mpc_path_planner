# MPC Path Planner

## Overview

This is a basic implementation of a model predictive control path planner in C++ using the *CppAD* and *Ipopt* libraries.

It uses ROS to communicate the position and velocity data from and to a turtlesim node.

## Instructions

### Build

You will need `termcolor` header-only library for colored terminal output (you can clone it from [Termcolor's GitHub repository](https://github.com/ikalnytskyi/termcolor)). Make sure the header file is in your include path.

Use `catkin_make` to build the node.

### Run

To run the program, first run the `main.launch` file, and then in a separate terminal run the `path_planner` node. You can control the obstacle turtle using *teleop_key* from the first terminal. The second terminal will be the user interface for publishing the destinations.
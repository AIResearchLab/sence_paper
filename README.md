# sence_robot

[![License](https://img.shields.io/badge/license-MIT-green)](./LICENSE)

## Overview

ROS meta-package for the Sence quadruped robot (created by Dylan Morley in 2022). Holding all the sence configuration information including micro controller communication and simulation execution.

This branch is for the ROS2 Humble/ Ignition Fortress version. It is dependent on the dynamixel-community hardware interface for serial communication.

Run the sence_poser action_menu node in combination with either the sence (hardware) or sence_gazebo (sim) launch files to run poses or sequences, or use the display launch file to view and move the URDF in Rviz2.
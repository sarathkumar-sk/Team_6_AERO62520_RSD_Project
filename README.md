
<img title="UoM_Logo"  src="/Images/Sensor/Uom.png"  width=40% height=auto>
# Team 6: Autonomous Mobile Manipulator (MSc Robotics)

![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E?style=for-the-badge&logo=ros&logoColor=white)
![Platform](https://img.shields.io/badge/Platform-Leo_Rover-orange?style=for-the-badge)
![Compute](https://img.shields.io/badge/Compute-Intel_NUC-0071C5?style=for-the-badge&logo=intel&logoColor=white)

## 📌 Project Overview
This repository contains the source code for **Team 6's Autonomous Mobile Manipulator**. The system is designed to autonomously explore a structured arena, detect coloured cubes using computer vision, and sort them into designated bins using a 6-DOF robotic arm.

The project integrates **SLAM (Simultaneous Localization and Mapping)**, **Navigation (Nav2)**, **Perception (YOLOv8 + RealSense)**, and **Manipulation (MoveIt 2)** into a unified distributed system running on **ROS 2 Jazzy**.

---

## 🤖 System Architecture

### Hardware
* **Mobile Base:** Leo Rover v1.8 (Raspberry Pi 4)
* **Main Compute:** Intel NUC (i7/i5) running Ubuntu 24.04
* **Sensors:**
    * RPLiDAR A2M12 (2D LiDAR for Navigation)
    * Intel RealSense D435 (RGB-D Camera for Perception)
* **Manipulator:** Elephant Robotics myCobot 280 Pi (6-DOF)

### Software Stack
* **Middleware:** ROS 2 Jazzy 
* **Navigation:** Nav2 Stack (AMCL, Costmap 2D, Smac Planner)
* **Manipulation:** MoveIt 2 (OMPL, KDL Kinematics)
* **Perception:** Ultralytics YOLOv8 + `realsense2_camera`
* **Communication:** FastDDS with `ROS_DOMAIN_ID=6`

---


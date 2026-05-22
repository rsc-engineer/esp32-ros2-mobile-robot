# ESP32 Autonomous Mobile Robot with micro-ROS

## Overview
Embedded control system for a differential drive mobile robot. The project bridges low-level hardware control on an ESP32 with high-level trajectory tracking and telemetry using ROS2 and micro-ROS over WiFi. 

## Software Architecture

### 1. Low-Level Firmware (ESP32 / Bare-Metal & RTOS concepts)
* **Hardware Interrupts:** Real-time encoder reading via `IRAM_ATTR` interrupt service routines for precise odometry.
* **Critical Sections:** Implementation of `portENTER_CRITICAL(&mux)` (Mutex) to prevent race conditions during atomic reads of volatile encoder counters.
* **PWM Motor Control:** Configured LEDC channels for smooth velocity control of the DC motors via L298N drivers.
* **Closed-Loop Control:** Independent PID controllers with anti-windup algorithms for precise wheel velocity regulation.
* **Odometry:** Real-time calculation of linear and angular velocities, transforming raw encoder counts to standard kinematics.

### 2. High-Level Control (ROS2)
* **micro-ROS Integration:** The ESP32 acts as a micro-ROS node, publishing real-time telemetry (`geometry_msgs/Pose2D`) and subscribing to velocity commands (`geometry_msgs/Twist`).
* **Path Following:** Implementation of a custom "Follow the Carrot" algorithm in a C++ ROS2 node (`carrot_node.cpp`) to track specific waypoints and compute optimal *v* and *w* commands based on the robot's heading and lookahead distance.

## Hardware Stack
* **Microcontroller:** ESP32
* **Actuators:** DC Motors with quadrature encoders
* **Motor Driver:** L298N
* **Communications:** WiFi (micro-ROS Agent)

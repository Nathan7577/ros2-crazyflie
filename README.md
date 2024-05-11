# Drone Control System

## Overview

This project aims to develop a system for controlling a Crazyflie drone to perform specific tasks autonomously. The primary objective is to enable the drone to take off, navigate to the position of a target, and land on it. To achieve this, the system integrates various components including the Crazyflie drone, an iRobot Create3 as the target platform, a Vicon motion tracking system for positional data, and a ROS2 node for communication and control.

## Objectives

- Develop a control system for the Crazyflie drone to autonomously navigate to a target position.
- Integrate the iRobot Create3 as a target platform for the drone to land on.
- Implement a ROS2 node to handle communication between the various components.
- Utilize a Vicon motion tracking system to provide accurate positional data for both the drone and the iRobot Create3.

## System Architecture

The system architecture consists of the following key components:

1. **Crazyflie Drone**: The drone platform that performs the autonomous flight tasks.
2. **iRobot Create3**: The target platform for the drone to land on, controlled remotely by the user.
3. **Vicon Motion Tracking System**: Provides positional data for both the drone and the iRobot Create3.
4. **ROS2 Node**: Responsible for communication between components and implementing the control logic.
5. **PI Controller**: Implements a Proportional-Integral (PI) controller with saturation for drone control.
6. **Error Computation**: Error terms are computed in the global frame and control inputs are rotated into the body frame of the drone.

## Control Strategy

The control strategy employed in this project utilizes a PI controller to regulate the drone's position and movement. The controller receives positional data from the Vicon motion tracking system and computes error terms relative to the target position. These error terms are then used to generate control inputs for the drone, taking into account saturation limits to ensure stable and safe flight.

## Future Work

While the current implementation achieves the basic objectives of autonomous flight and target landing, there are several areas for future improvement and expansion:

- **Obstacle Avoidance**: Integrate obstacle detection and avoidance algorithms to enhance the drone's navigation capabilities.
- **Real-Time Trajectory Planning**: Implement algorithms for real-time trajectory planning to optimize flight paths and improve efficiency.
- **Enhanced User Interface**: Develop a user-friendly interface for remote control and monitoring of the system.
- **Multi-Drone Coordination**: Explore methods for coordinating multiple drones to perform collaborative tasks.

## Conclusion

The development of this drone control system represents a significant step towards enabling autonomous flight and precise task execution. By integrating advanced control algorithms with state-of-the-art hardware and sensing technologies, the system demonstrates the potential for unmanned aerial vehicles to perform a wide range of tasks in various environments.


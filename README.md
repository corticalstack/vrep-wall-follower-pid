# 🤖 V-REP Wall Follower with PID Control

A Python implementation of wall-following algorithms for the Pioneer P3DX robot in the V-REP (CoppeliaSim) simulation environment, featuring PID control for smooth navigation.

## 📝 Description

This project implements various wall-following algorithms for autonomous robot navigation in the V-REP (now known as CoppeliaSim) simulation environment. The primary focus is on a PID (Proportional-Integral-Derivative) controller that enables a Pioneer P3DX robot to follow walls smoothly while maintaining a safe distance.

The implementation includes several wall-following methods:
- PID control-based wall following
- Minimum sensor reading-based wall following
- Multi-sensor weighted approach

## 🔧 Prerequisites

- [Python 3.x](https://www.python.org/downloads/)
- [V-REP/CoppeliaSim](https://www.coppeliarobotics.com/downloads) (Educational or Pro version)
- V-REP Python API (included with V-REP installation)
- Python packages:
  - numpy
  - math
  - logging

## ✨ Features

- **Multiple Wall-Following Algorithms**:
  - PID controller with tunable parameters
  - Minimum sensor reading approach
  - Multi-sensor weighted approach
- **Comprehensive Sensor Integration**:
  - Ultrasonic proximity sensors
  - Compass for orientation
  - Odometry for distance tracking
- **Data Logging and Analysis**:
  - Detailed logging of robot state
  - CSV output for position, motor velocity, and error history
  - Performance statistics (distance traveled, average speed, etc.)
- **Configurable Scenarios**:
  - Customizable robot tasks (turn, move, wall follow)
  - Adjustable parameters (minimum distance, maximum distance, velocity)

## 🚀 Setup Guide

1. Install V-REP/CoppeliaSim and ensure the Python API is properly configured
2. Clone this repository to your local machine
3. Create `logs` and `output` directories in the project root:
   ```bash
   mkdir logs output
   ```
4. Open V-REP/CoppeliaSim and load a scene with the Pioneer P3DX robot

## 📋 Usage

1. Start V-REP/CoppeliaSim and load a scene with the Pioneer P3DX robot
2. Run the scenario script:
   ```bash
   python scenario.py
   ```
3. The robot will execute the defined tasks:
   - Turn to a specified orientation
   - Move in a straight line until near a wall
   - Follow the wall perimeter for one complete loop
   - Stop

## 🏗️ Architecture

The project is organized into several Python modules:

- **scenario.py**: Main entry point that defines the robot's tasks and parameters
- **controller.py**: Manages the connection to V-REP and executes the control loop
- **robots.py**: Implements the Pioneer P3DX robot with various movement methods
- **sensors.py**: Provides classes for proximity sensors and compass
- **helper.py**: Contains utility functions for V-REP interaction
- **logger.py**: Handles logging of robot state and events

### Control Loop

The main control loop follows a sense-think-act paradigm:
1. **Sense**: Update robot state (position, proximity, compass, odometry)
2. **Think**: Process sensor data and determine appropriate action
3. **Act**: Execute movement commands based on the current task

### PID Controller

The PID wall-following algorithm uses:
- **Proportional (P)**: Responds to the current error (distance from wall)
- **Integral (I)**: Accounts for accumulated error over time
- **Derivative (D)**: Responds to the rate of change of error

Default PID parameters:
- kp: 7.29
- ki: 0.0
- kd: -7.0

## 📊 Data Output

The simulation generates several CSV files in the `output` directory:
- **abs_pos_all.csv**: Absolute position of the robot throughout the simulation
- **motor_all_v**: Motor velocity data
- **error_history**: PID controller error history

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

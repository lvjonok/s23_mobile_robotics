# S23 Autonomous mobile robotics course

This repository contains the code for homeworks. Meant to be used together with [original repostory](https://github.com/GPrathap/autonomous_mobile_robots)

## Homeworks

1. [Homework 1](s23_mobile_robotics/homework1.md) - comparison of analytical and estimated trajectories. [Report](homework1.pdf))

2. [Homework 2](s23_mobile_robotics/homework2.py) - implementation of simple path tracking strategy through waypoints. [Report](homework2.pdf))

3. [Homework 3](s23_mobile_robotics/homework3.py) - implementation of lateral control for path tracking. [Report](homework3.pdf)

## Usage

1. Install ROS2

1. Clone the repository

```bash
 mkdir -p ~/ws/src
 cd ~/ws/src
 git clone https://github.com/GPrathap/autonomous_mobile_robots.git
 git clone https://github.com/lvjonok/s23_mobile_robotics.git
```

2. Build the workspace

```bash
 cd ~/ws
 colcon build
```

3. Source the workspace

```bash
 source ~/ws/install/setup.bash
```

4. Instructions on running launches with gazebo and rviz could be found in original repository.

5. All scripts in this repository will be built as executables and available after sourcing the environment.

   Run the script with `ros2 run s23_mobile_robotics <script_name>`

   For example: `ros2 run s23_mobile_robotics hw1`

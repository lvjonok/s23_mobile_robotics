# S23 Autonomous mobile robotics course

This repository contains the code for homeworks. Meant to be used together with [original repostory](https://github.com/GPrathap/autonomous_mobile_robots)

## Homeworks

1. [Homework 1](homework1.md)

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

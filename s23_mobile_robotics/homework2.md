# Homework 2 - Lev Kozlov

In this assignment we implement simple path tracking strategy.
It accepts a list of waypoints and performs tracking of the each segment consequently.

## How to run:

```bash
ros2 run s23_mobile_robotics hw2
```

In `rviz` directory you may find configuration file for `rviz` to visualize the robot with both target and current trajectory.

## Algorithm

Probably there is no need to explain the algorithm itself, because it was given within assignment description.

The most important part is to calculate derivation of linear and angular velocities.

As simplification I added `(0, 0)` as starting point to the trajectory. It allowed to follow first waypoint at `i = 0`.

Also check on which segment we are:

$$u = \frac{v^T r}{v^T v}$$

Does not cover zero division case and I moved to the next segment if it occured.

## Observations:

Feedforward control is not enough to properly track the trajectory.
The most obvious problem is turning. We integrate angular velocity and it may result in some errors where robot misses his actual angle.

Examples with different trajectories and results:

![Trajectory 1](data/hw2_images/result.png)

![Trajectory 2](data/hw2_images/result2.png)

![Trajectory 3](data/hw2_images/result3.png)

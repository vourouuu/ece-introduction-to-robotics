# Webots Controller Files
Path: `homeworks/homework-3/homework_3/controllers/PandaController`

---

## Core Scripts
There are two main scripts used for the simulation:

### 1. Trajectory and Velocity Control
Filename: `trajectory-velocity-control-panda-7DOF.py` 

This script is responsible for:
* Calculating the required **trajectories**.
* Generating the **velocity commands** for the robot arm.

### 2. Robot Controller
Filename: `PandaController.py`  

This script handles the execution of the movement by:
* Reading the generated `.txt` files containing the velocity lists for each joint per trajectory.
* Assigning these values to the respective variables (`v1`, `v2`, `v3`, `v4`, and `v5`).
* Commanding the arm to execute the calculated velocities in the simulation environment.
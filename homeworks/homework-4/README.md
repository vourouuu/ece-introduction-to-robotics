# Controller Files Overview
Path: `homeworks/homework-4/homework_4/controllers/ArmController`

---

## Core Scripts
The project relies on two primary script files to handle trajectory calculation and robot execution:

### 1. Dynamics and Trajectory Calculation
Filename: `ArmControlWithDynamics.py`  

This script is responsible for the mathematical foundation of the movement:
* **Sinusoidal Trajectory:** Calculates the joint angles required for the sinusoidal motion.
* **Normalization:** Scales the calculated angles within the intervals `[-π/2, π/2]` and `[-π, π]`, depending on the specific joint's constraints.
* **Data Export:** Saves the final normalized angles into the `q1.txt` file for use by the controller.

### 2. Robot Controller
Filename: `ArmController.py`  

This script manages the physical simulation within Webots:
* **Data Retrieval:** Reads the joint angle lists directly from the `q1.txt` file.
* **Simulation Execution:** Commands the robotic arm to follow the predefined angles to achieve the desired simulation behavior.
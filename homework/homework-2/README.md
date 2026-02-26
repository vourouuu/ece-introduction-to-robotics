# Webots Simulation Guide

## Opening the World and Starting Simulation
Open the world file:

`File > Open World > homeworks/homework-2/homework_2/worlds/homework_2.wbt`

To view the controller codes for the arm and world objects:

`Tools > Text Editor`

If the files `SupervisorController.py` and `PandaController.py` are not already open, click the folder icon in the Text Editor and navigate to:

* `homeworks/homework-2/homework_2/controllers/PandaController.py`
* `homeworks/homework-2/homework_2/controllers/SupervisorController.py`

---

## Simulation Description
Upon opening the world, all objects (robot arm, box, cylinder) are initially located at position **(0,0,0)** and oriented according to the **world frame**.

Once the simulation starts:
* All objects move to their correct predefined positions as specified.
* The robot arm moves to the position determined during the problem-solving process.
* The **transformation matrices** of the objects are displayed in the console.

### Movement Workflow
1. **Home Configuration:** The arm starts at its initial state.
2. **Grasping:** It moves over the cylinder and grips it.
3. **Return to Home:** It returns to the Home Configuration to avoid collisions with the box or floor, as obstacle avoidance parameters were not used.
4. **Release:** It moves to the target position and releases the cylinder into the hole.

---

## Inverse Kinematics (IK) Calculation
By running the `ik-fk-panda-7DOF.py` script, we obtain the joint angles for the following IK transitions:
* From **Home Configuration** to the position over the cylinder.
* From the position over the cylinder back to **Home Configuration**.
* From **Home Configuration** to the position over the hole.

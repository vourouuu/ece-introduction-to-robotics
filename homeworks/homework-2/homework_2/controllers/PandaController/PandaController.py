import math
import numpy as np
from controller import Robot, Supervisor

TIME_STEP  = 32

block_positions = [[0.0, 0.0, 0.0, -0.01, 0.0, -0.01, -0.01],                     # Home configuration
                   [0.3276, 2.0044, -5.6941, 0.5694, 5.7539, -4.7653, -5.4748],   # Gripper above the cylinder...(ik1)
                   [5.3702, -0.5697, -6.1532, -0.9682, 0.1771, -5.8751, -0.6405], # Return to home configuration.(ik2)
                   [4.3904, -1.7187, -2.4207, 0.3779, 0.9390, -2.1351, -0.9485]]  # Gripper above the hole.......(ik3)

JOINT1, JOINT2, JOINT3, JOINT4, JOINT5, JOINT6, JOINT7, FINGER = range(8)
BLOCK0, BLOCK1, BLOCK2, BLOCK3 = range(4)
OPEN_HAND, CLOSE_HAND = range(2)

class PandaController(Supervisor):
    def __init__(self):
        super().__init__()
        self.motors = [self.getDevice(f"panda_joint{i+1}") for i in range(7)]
        self.motors.append(self.getDevice("panda_finger::right"))
        self.arm_node = self.getFromDef("ARM")

    def hand_control(self, command):
        self.motors[FINGER].setPosition(0.035 if command == OPEN_HAND else 0.009)
        self.step(TIME_STEP * 10)

    def run(self):
        self.hand_control(OPEN_HAND)
        self.step(TIME_STEP * 40)
        
        # Home configuration
        self.motors[JOINT1].setPosition(block_positions[BLOCK0][0])
        self.motors[JOINT2].setPosition(block_positions[BLOCK0][1])
        self.motors[JOINT3].setPosition(block_positions[BLOCK0][2])
        self.motors[JOINT4].setPosition(block_positions[BLOCK0][3])
        self.motors[JOINT5].setPosition(block_positions[BLOCK0][4])
        self.motors[JOINT6].setPosition(block_positions[BLOCK0][5])
        self.motors[JOINT7].setPosition(block_positions[BLOCK0][6])

        self.step(TIME_STEP * 40) # delay
        
        # Gripper above the cylinder
        self.motors[JOINT7].setPosition(block_positions[BLOCK1][6])
        self.step(TIME_STEP * 40)
        self.motors[JOINT1].setPosition(block_positions[BLOCK1][0])
        self.step(TIME_STEP * 40)
        self.motors[JOINT4].setPosition(block_positions[BLOCK1][3])
        self.step(TIME_STEP * 40)
        self.motors[JOINT3].setPosition(block_positions[BLOCK1][2])
        self.step(TIME_STEP * 40)
        self.motors[JOINT5].setPosition(block_positions[BLOCK1][4])
        self.step(TIME_STEP * 40)
        self.motors[JOINT6].setPosition(block_positions[BLOCK1][5])
        self.step(TIME_STEP * 40)
        self.motors[JOINT2].setPosition(block_positions[BLOCK1][1])
        self.step(TIME_STEP * 70)
        
        # Return to home configuration
        self.motors[JOINT2].setPosition(block_positions[BLOCK2][1])
        self.step(TIME_STEP * 40)
        self.motors[JOINT6].setPosition(block_positions[BLOCK2][5])
        self.step(TIME_STEP * 40)
        self.motors[JOINT7].setPosition(block_positions[BLOCK2][6])
        self.step(TIME_STEP * 40)
        self.motors[JOINT5].setPosition(block_positions[BLOCK2][4])
        self.step(TIME_STEP * 40)
        self.motors[JOINT4].setPosition(block_positions[BLOCK2][3])
        self.step(TIME_STEP * 40)
        self.motors[JOINT3].setPosition(block_positions[BLOCK2][2])
        self.step(TIME_STEP * 40)
        self.motors[JOINT1].setPosition(block_positions[BLOCK2][0])
        self.step(TIME_STEP * 40)
        
        # Gripper above the hole
        self.motors[JOINT3].setPosition(block_positions[BLOCK3][2])
        self.step(TIME_STEP * 40)
        self.motors[JOINT4].setPosition(block_positions[BLOCK3][3])
        self.step(TIME_STEP * 40)
        self.motors[JOINT5].setPosition(block_positions[BLOCK3][4])
        self.step(TIME_STEP * 40)
        self.motors[JOINT7].setPosition(block_positions[BLOCK3][6])
        self.step(TIME_STEP * 40)
        self.motors[JOINT1].setPosition(block_positions[BLOCK3][0])
        self.step(TIME_STEP * 40)
        self.motors[JOINT2].setPosition(block_positions[BLOCK3][1])
        self.step(TIME_STEP * 40)
        self.motors[JOINT6].setPosition(block_positions[BLOCK3][5])
        self.step(TIME_STEP * 40)
        

controller = PandaController()
controller.run()
# * File: PandaController.py
# * Date: 12/01/2025
# * Description: Manipulator control with velocity commands
# * Author: Vrachoriti Alexandra

import math
import numpy as np
import json
from controller import Robot, Supervisor

TIME_STEP  = 100
INF = float('inf')

block_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

vel_inf = [INF, INF, INF, INF, INF, INF, INF]

JOINT1, JOINT2, JOINT3, JOINT4, JOINT5, JOINT6, JOINT7, FINGER = range(8)

class PandaController(Supervisor):
    def __init__(self):
        super().__init__()
        self.motors = [self.getDevice(f"panda_joint{i+1}") for i in range(7)]
        self.finger = self.getDevice("panda_finger::right")
        self.arm_node = self.getFromDef("ARM")
    
    def read_file_to_list(self, filename):
        try:
            with open(filename, 'r') as file:
                content = file.read()
                return json.loads(content)
        except Exception as e:
            print(f"Error reading {filename}: {e}")
            return [[]]

    def load_velocities(self):
        global v1, v2, v3, v4, v5
        
        v1 = self.read_file_to_list('v1.txt')
        v2 = self.read_file_to_list('v2.txt')
        v3 = self.read_file_to_list('v3.txt')
        v4 = self.read_file_to_list('v4.txt')
        v5 = self.read_file_to_list('v5.txt')

    def set_motor_velocities(self, velocities):
        for i in range(len(self.motors)):
            self.motors[i].setVelocity(velocities[i])
    
    def grab(self, command):
        if(command == 'OPEN'):
            self.finger.setPosition(0.035)
        else:
            self.finger.setPosition(0.012)
            
    def init(self):
        self.motors[JOINT1].setPosition(block_positions[0])
        self.motors[JOINT2].setPosition(block_positions[1])
        self.motors[JOINT3].setPosition(block_positions[2])
        self.motors[JOINT4].setPosition(block_positions[3])
        self.motors[JOINT5].setPosition(block_positions[4])
        self.motors[JOINT6].setPosition(block_positions[5])
        self.motors[JOINT7].setPosition(block_positions[6])
        self.step(TIME_STEP*5)
        
        self.grab('OPEN')
        self.step(TIME_STEP)
    
    def control_with_velocities(self):
        # Change control mode: position ---> velocity
        for i in range(len(self.motors)):
            self.motors[i].setPosition(float('inf'))
        
        # Velocity commands: home configuration ---> intermediate position (1) 
        for i in range(len(v1)):
            self.set_motor_velocities(v1[i])
            self.step(TIME_STEP)
        
        # Velocity commands: intermediate position (1) ---> above the cylinder
        for i in range(len(v2)):
            self.set_motor_velocities(v2[i])
            self.step(TIME_STEP)
        
        # Grab the cylinder 
        self.grab('CLOSE')
        self.step(TIME_STEP)
        
        # Velocity commands: above the cylinder ---> intermediate position (1)
        for i in range(len(v3)):
            self.set_motor_velocities(v3[i])
            self.step(TIME_STEP)
            
        # Velocity commands: intermediate position (1) ---> intermediate position (2)
        for i in range(len(v4)):
            self.set_motor_velocities(v4[i])
            self.step(TIME_STEP)
        
        # Velocity commands: intermediate position (2) ---> above the hole
        for i in range(len(v5)):
            self.set_motor_velocities(v5[i])
            self.step(TIME_STEP)
        
        # Release the cylinder 
        self.grab('OPEN')
        self.step(TIME_STEP)
     
    def run(self):
        self.load_velocities()
        self.init()
        self.control_with_velocities()
        
controller = PandaController()
controller.run()

# * File: ArmController.py
# * Date: 02/02/2025
# * Description: Manipulator control with position commands
# * Author: Vrachoriti Alexandra

import math
import numpy as np
import json
from controller import Robot, Supervisor

JOINT1, JOINT2, JOINT3, JOINT4 = range(4)

class ArmController(Robot):
    def __init__(self):
        super().__init__()
        self.motors = [self.getDevice(f"arm_joint_{i+1}") for i in range(4)]
        
    def read_file_to_list(self, filename):
        try:
            with open(filename, 'r') as file:
                content = file.read()
                return json.loads(content)
        except Exception as e:
            print(f"Error reading {filename}: {e}")
            return [[]]

    def load_positions(self):
        q = self.read_file_to_list('q1.txt')
        
        return q
            
    def set_q(self, q):
        for i in range(len(q)):
            self.motors[JOINT1].setPosition(q[i][0])
            self.motors[JOINT2].setPosition(q[i][1])
            self.motors[JOINT3].setPosition(q[i][2])
            self.motors[JOINT4].setPosition(q[i][3])
            
            self.step(60)
        
    def run(self):
        q = self.load_positions()
        self.set_q(q)
        
controller = ArmController()
controller.run()
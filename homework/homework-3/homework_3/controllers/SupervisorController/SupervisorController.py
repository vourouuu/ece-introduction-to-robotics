# * File: SupervisorController.py
# * Date: 12/01/2025
# * Description: Print Cylinder orientation, Box orientation and World orientation
# * Author: Vrachoriti Alexandra

import math
import numpy as np
from controller import Robot, Supervisor

supervisor = Supervisor()

PI = np.pi

def Rx(theta):
    theta = theta * (PI / 180.0) # degrees to rad
    
    R = np.array([
        [1.0, 0.0, 0.0],
        [0.0, np.cos(theta), -np.sin(theta)],
        [0.0, np.sin(theta), np.cos(theta)]
    ])
    
    return R 

def Ry(theta):
    theta = theta * (PI / 180.0) # degrees to rad
    
    R = np.array([
        [np.cos(theta), 0.0, np.sin(theta)],
        [0.0, 1.0, 0.0],
        [-np.sin(theta), 0.0, np.cos(theta)]
    ])
    
    return R

def Rz(theta):
    theta = theta * (PI / 180.0) # degrees to rad
    
    R = np.array([
        [np.cos(theta), -np.sin(theta), 0.0],
        [np.sin(theta), np.cos(theta), 0.0],
        [0.0, 0.0, 1.0]
    ])
    
    return R

def T(R, p):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    
    return T
   
def print_matrix(matrix, name):
    print(f"\n\n> {name}'s Transformation Matrix:")

    for row in matrix:
        print(" ".join(f"{val:7.3f}" for val in row))
    
    return 0

def main():    
    # "RectangleArena" node (World)
    R_w = np.dot(np.dot(Rx(0.0), Ry(0.0)), Rz(0.0))
    p_w = [0.0, 0.0, 0.0]
    T_w = T(R_w, p_w)
    
    # "red-box" node
    R_wb = np.dot(np.dot(np.dot(Rx(0.0), Ry(0.0)), Rz(0.0)), R_w)
    p_wb = [0.5, 0.5, 0.1]
    T_wb = T(R_wb, p_wb)
    
    # "orange-cylinder" node
    R_wc = np.dot(np.dot(np.dot(Rx(0.0), Ry(-90)), Rz(0.0)), R_w)
    p_wc = [0.695, 0.095, 0.025]
    T_wc = T(R_wc, p_wc)
    
    # Print matrices
    print_matrix(T_w, "World")
    print_matrix(T_wb, "Box")
    print_matrix(T_wc, "Cylinder")

if __name__ == "__main__":
    main()
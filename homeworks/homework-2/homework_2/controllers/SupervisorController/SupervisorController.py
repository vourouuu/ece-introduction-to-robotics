# * File: pySupervisorController.py
# * Date: 09/12/2024
# * Description: Controls Cylinder's and Box's orientation
# * Author: Vrachoriti Alexandra

import math
import numpy as np
from controller import Robot, Supervisor

supervisor = Supervisor()

TIME_STEP  = 32
PI = np.pi

def deg_to_rad(degrees):
    return degrees * PI / 180.0

def Rx(theta_deg):
    theta_rad = deg_to_rad(theta_deg)
    
    R = np.array([
        [1.0, 0.0, 0.0],
        [0.0, np.cos(theta_rad), -np.sin(theta_rad)],
        [0.0, np.sin(theta_rad), np.cos(theta_rad)]
    ])
    
    return R 

def Ry(theta_deg):
    theta_rad = deg_to_rad(theta_deg)
    
    R = np.array([
        [np.cos(theta_rad), 0.0, np.sin(theta_rad)],
        [0.0, 1.0, 0.0],
        [-np.sin(theta_rad), 0.0, np.cos(theta_rad)]
    ])
    
    return R

def Rz(theta_deg):
    theta_rad = deg_to_rad(theta_deg)
    
    R = np.array([
        [np.cos(theta_rad), -np.sin(theta_rad), 0.0],
        [np.sin(theta_rad), np.cos(theta_rad), 0.0],
        [0.0, 0.0, 1.0]
    ])
    
    return R

def T(R, p):
    transf_matrix = np.eye(4)
    transf_matrix[:3, :3] = R
    transf_matrix[:3, 3] = p
    
    return transf_matrix

def multiply_matrices(matrix1, matrix2):
    return np.dot(matrix1, matrix2)
    
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
    
    # Move the "red-box" node to the correct position
    BoxNode = supervisor.getFromDef("red-box")
    BoxNode.getField("translation").setSFVec3f(p_wb)
    
    # Move the "orange-cylinder" node to the correct position
    CylinderNode = supervisor.getFromDef("orange-cylinder")
    CylinderNode.getField("translation").setSFVec3f(p_wc)
    
    # Rotate the "red-box" node
    BoxNode.getField("rotation").setSFRotation([1, 0, 0, deg_to_rad(0)])  # rot. axis: x, rotation: 0 rad
    
    # Rotate the "orange-cylinder" node
    CylinderNode.getField("rotation").setSFRotation([0, 1, 0, deg_to_rad(-90)])  # rot axis: y, rotation: -90 degrees
    
    # Print matrices
    print_matrix(T_w, "World")
    print_matrix(T_wb, "Box")
    print_matrix(T_wc, "Cylinder")
    
    supervisor.step(TIME_STEP * 20)

if __name__ == "__main__":
    main()
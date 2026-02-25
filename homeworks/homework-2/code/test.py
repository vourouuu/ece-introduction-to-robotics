import numpy as np
from scipy.linalg import logm
import matplotlib.pyplot as plt

np.set_printoptions(precision=4, suppress=True, floatmode='fixed')

PI = np.pi

#### Screw Axis ####
r = np.array([
    [0.0, 0.0, 1.0], #1 OK
    [0.0, 1.0, 0.0], #2 OK
    [0.0, 1.0, 0.0], #3 OK
    [1.0, 0.0, 0.0], #4 OK
    [0.0, 0.0, 1.0], #5 OK
    [1.0, 0.0, 0.0], #6 OK
    [0.0, 0.0, 1.0]  #7 OK
])

d1 = 169.5 / 1000.0
d3 = 115.5 / 1000.0
d5 = 127.83 / 1000.0
d7 = 65.98 / 1000.0
q = np.array([
    [0.0, 0.0, 0.0],
    [0.0, 0.0, d1],
    [0.0, 0.0, d1+d3],
    [0.088, 0.0, d1+d3],
    [0.0, 0.0, d1+d3+d5],
    [0.0, 0.0, d1+d3+d5],
    [0.088, 0.0, d1+d3+d5+d7]
])

S = np.zeros((7, 6))

for i in range(S.shape[0]):
    r_i = r[i]
    q_i = q[i]
    p_i = -np.cross(r_i, q_i)
    S[i, :3] = r_i
    S[i, 3:] = p_i
####################

def deg_to_rad(degrees):
    return degrees * (PI / 180.0)

def normalize(vector):
    # Normalization of theta (theta is a vector (1 x 7) and represents the returned value from IK())
    for i in range(len(vector)):
        if(abs(vector[i]) > 2*PI):
            if(vector[i] < 0):
                a = -vector[i]
                a = a%(2*PI)
                vector[i] = -a
            else: vector[i] = vector[i]%(2*PI)

    for i in range(len(vector)):
        if((i == 3 or i == 1) and abs(vector[i]) > PI):
            a = 2*PI - abs(vector[i])
            if(vector[i] < 0): vector[i] = a
            else: vector[i] = -a

    return vector

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

def skew_symmetric(vector):
    if (vector.shape[0] == 3):
        return np.array([
            [0, -vector[2], vector[1]],
            [vector[2], 0, -vector[0]],
            [-vector[1], vector[0], 0]
        ])
    else:
        w = vector[:3]
        v = vector[3:]
        
        # skew symmetric of w
        w_skew_symmetric = np.array([
            [0, -w[2], w[1]],
            [w[2], 0, -w[0]],
            [-w[1], w[0], 0]
        ])
        
        # skew symmetric matrix
        skew_symmetric_matrix = np.zeros((4, 4))
        skew_symmetric_matrix[:3, :3] = w_skew_symmetric
        skew_symmetric_matrix[:3, 3] = v
        
        return skew_symmetric_matrix

def exp_rot(r, theta_rad):
    r_hat = skew_symmetric(r)
    transp_r = r.reshape(1, -1).T # horizontal vector ---> vertical vector
    
    matrix = np.eye(3) * np.cos(theta_rad) + (1 - np.cos(theta_rad)) * (transp_r * r) + np.sin(theta_rad) * r_hat

    return matrix

def exp_pose(S, theta_rad):
    r = S[:3]
    p = S[3:]

    transp_p = p.reshape(1, -1).T # horizontal vector ---> vertical vector
    r_hat = skew_symmetric(r)
    
    matrix = np.zeros((4, 4))
    matrix[0:3, 0:3] = exp_rot(r, theta_rad)
    matrix[0:3, 3:] = (np.eye(3) * theta_rad + (1 - np.cos(theta_rad)) * r_hat + (theta_rad - np.sin(theta_rad)) * (r_hat @ r_hat)) @ transp_p
    matrix[3, :] = [0.0, 0.0, 0.0, 1.0]

    return matrix

def Ad(T):
    R = T[0:3, 0:3]
    p = T[0:3, 3]

    p_hat = skew_symmetric(p)
    
    matrix = np.zeros((6, 6))
    matrix[0:3, 0:3] = R
    matrix[3:, 0:3] = p_hat @ R
    matrix[3:, 3:] = R

    return matrix

def pseudo_inverse(J):
    J_T = J.T
    matrix = J_T @ np.linalg.inv(J @ J_T) # n (= 7) > m (= 6)

    return matrix

def FK(S, theta, M):
    Tw_ee = np.eye(4)

    for i in range(len(theta)):
        Tw_ee = Tw_ee @ exp_pose(S[i], theta[i])

    Tw_ee = Tw_ee @ M

    return Tw_ee

def IK(goal_pos, initial_theta, M):
    theta_history = []
    norm_history = []

    theta = initial_theta

    Vw = np.zeros((6, 1)) # Vw (Space Twist)
    delta_theta = np.zeros((7, 1))

    iterations = 0
    # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

    while(np.linalg.norm(Vw) >= 1e-3 or np.linalg.norm(Vw) == 0):
        iterations += 1

        ############################# Jacobian #############################
        Jw_T = []  # Transpose Space Jacobian (Jw_T) ---> dimensions: (7 x 6)
        exp_p = [] # Exponential Pose

        for i in range(len(theta)):
            exp_p.append(exp_pose(S[i], theta[i]))

        Jw_T.append(S[0, :])
        Jw_T.append(Ad(exp_p[0]) @ (S[1, :]))

        x = exp_p[0]
        for i in range(1, len(exp_p)-1):
            y = exp_p[i]
            x = x @ y
            z = Ad(x) @ S[i+1, :]

            Jw_T.append(z)

        Jw_T = np.array(Jw_T) # dimensions: (7 x 6)
        # print(f"Jw_T = {Jw_T}")
        Jw = Jw_T.T.copy()    # dimensions: (6 x 7)
        Jw_T = list(Jw_T)
        Jw_pseudo_inv = pseudo_inverse(Jw)
        ####################################################################

        ########################## Neuton-Raphson ##########################
        current_pos = np.linalg.inv(FK(S, theta, M))
        Vw_hat = logm(goal_pos @ current_pos)

        Vw[0] = -Vw_hat[1, 2]
        Vw[1] = Vw_hat[0, 2]
        Vw[2] = -Vw_hat[0, 1]
        Vw[3] = Vw_hat[0, 3]
        Vw[4] = Vw_hat[1, 3]
        Vw[5] = Vw_hat[2, 3]

        delta_theta = Jw_pseudo_inv @ Vw
        theta += delta_theta.T.flatten()
        ####################################################################

        theta_history.append(normalize(theta).copy())
        norm_history.append(np.linalg.norm(Vw))

    ################################ Plots #################################
    theta_history = np.array(theta_history)    # red
    theta_history = theta_history * (180 / PI) # degrees
    norm_history = np.array(norm_history)

    fig1 = plt.figure(figsize = (16, 9))
    for i in range(7):
        plt.subplot(3, 3, i+1)
        plt.plot(theta_history[:, i])
        plt.title(f'Joint {i+1} Angle Evolution')
        plt.xlabel('Iterations')
        plt.ylabel('Theta (degrees)')
        plt.grid(True)
    
    plt.tight_layout()
    plt.show()

    fig2 = plt.figure(figsize = (16, 9))
    plt.plot(norm_history)
    plt.title(f'Norm history')
    plt.xlabel('Iterations')
    plt.ylabel('||Vw||')
    plt.grid(True)
    plt.show()
    ########################################################################
    
    print("> Iterations = ", iterations)

    return theta

def main():
    Rw = np.eye(3)
    Rw_base = Rw
    Rw_xF = np.dot(Rw_base, np.dot(np.dot(Rx(180.0), Ry(0.0)), Rz(0.0)))
    Rw_ee = np.dot(Rw_xF, np.dot(np.dot(Rx(0.0), Ry(0.0)), Rz(-45.0)))

    pw_ee = [0.088, 0.0, 0.823]
    M = T(Rw_ee, pw_ee)
    print("> Transformation of the end-effector frame when all joints are in the home configuration (M):\n", M, "\n")

    print("-------------------------------------------------------------------------------------------\n")
    Rw_ee_des1 = np.dot(np.dot(np.dot(Rx(0.0), Ry(0.0)), Rz(-45.0)), Rw_ee)
    pw_ee_des1 = [0.695, 0.095, 0.02]
    Tw_ee_des1 = T(Rw_ee_des1, pw_ee_des1)
    print("> Desired pose of the end-effector such that is able to grasp the cylinder (wrt world frame):\n", Tw_ee_des1, "\n")

    initial_theta = [0.0, 0.0, 0.0, -0.01, 0.0, -0.01, -0.01] # home configuration
    print("> Initial theta (1): ", initial_theta, "\n")

    ik1 = IK(Tw_ee_des1, initial_theta, M)
    normalized_ik1 = normalize(ik1)

    print("> Inverse Kinematics (1): ", normalized_ik1)

    fk1 = FK(S, normalized_ik1, M) # or fk1 = FK(S, ik1, M)
    print(">  Foward Kinematics (1):\n", fk1, "\n")

    #########################################################################################################################
    
    print("-------------------------------------------------------------------------------------------\n")
    print("> Desired pose of the end-effector such that returns to the home configuration (wrt world frame):\n", M, "\n")
    print("> Initial theta (2): ", normalized_ik1, "\n")
    
    ik2 = IK(M, normalized_ik1, M)
    normalized_ik2 = normalize(ik2)

    print("> Inverse Kinematics (2): ", normalized_ik2)

    fk2 = FK(S, normalized_ik2, M) # or fk2 = FK(S, ik2, M)
    print(">  Foward Kinematics (2):\n", fk2, "\n")

    #########################################################################################################################
    
    print("-------------------------------------------------------------------------------------------\n")
    
    Rw_ee_des2 = np.dot(np.dot(np.dot(Rx(0.0), Ry(-90.0)), Rz(-45.0)), Rw_ee)
    pw_ee_des2 = [0.5, 0.5, 0.325]
    Tw_ee_des2 = T(Rw_ee_des2, pw_ee_des2)
    print("> Desired pose of the end-effector such that the cylinder is positioned in the “placing” position above the hole:\n", Tw_ee_des2, "\n")
    print("> Initial theta (3): ", normalized_ik2, "\n")
    
    ik3 = IK(Tw_ee_des2, normalized_ik2, M)
    normalized_ik3 = normalize(ik3)

    print("> Inverse Kinematics (3): ", normalized_ik3)

    fk3 = FK(S, normalized_ik3, M) # or fk3 = FK(S, ik3, M)
    print(">  Foward Kinematics (3):\n", fk3, "\n")

if __name__ == "__main__":
    main()


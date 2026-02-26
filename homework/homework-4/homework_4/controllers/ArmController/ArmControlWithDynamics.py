from scipy.linalg import logm
import math
import numpy as np
import matplotlib.pyplot as plt
from urdfpy import URDF

np.set_printoptions(precision=4, suppress=True, floatmode='fixed')

np.set_printoptions(linewidth=200)
robot = URDF.load('arm.urdf')

DoF = 4 
TOTAL_TIME = 0.45
TIME_STEP = 0.01

def normalize(vector):
    vector = vector.copy()

    for i in range(len(vector)):
        if(abs(vector[i]) > 1.57079):
            if(vector[i] < 0):
                a = -vector[i]
                a = a % np.pi
                vector[i] = -a

                if(abs(vector[i]) > 1.57079): vector[i] = -1.5707
            else:
                a = vector[i] % np.pi
                vector[i] = a

                if(abs(vector[i]) > 1.57079): vector[i] = 1.5707
                
    return vector

def T(R, p):                               # lecture  2, page 13
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    
    return T

def hat(vector):                           # lecture  3, page  5 - lecture 4, page 14
    if(vector.shape[0] == 3):
        vector_hat = np.array([
            [0, -vector[2], vector[1]],
            [vector[2], 0, -vector[0]],
            [-vector[1], vector[0], 0]
        ]) 
    else:
        w = vector[:3]
        v = vector[3:]
        
        # skew symmetric of w
        w_hat = np.array([
            [0, -w[2], w[1]],
            [w[2], 0, -w[0]],
            [-w[1], w[0], 0]
        ])
        
        # skew symmetric matrix
        vector_hat = np.zeros((4, 4))
        vector_hat[:3, :3] = w_hat
        vector_hat[:3, 3] = v
   
    return vector_hat

def unhat(vector_hat):                     # lecture  3, page  5 - lecture 4, page 14
    if(vector_hat.shape[0] == 3):
        vector_unhat = np.zeros((3, 1))

        vector_unhat[0] = vector_hat[2, 1]
        vector_unhat[1] = vector_hat[0, 2]
        vector_unhat[2] = vector_hat[1, 0]
    else:
        vector_unhat = np.zeros((6, 1))

        vector_unhat[0] = vector_hat[2, 1]
        vector_unhat[1] = vector_hat[0, 2]
        vector_unhat[2] = vector_hat[1, 0]
        vector_unhat[3] = vector_hat[0, 3]
        vector_unhat[4] = vector_hat[1, 3]
        vector_unhat[5] = vector_hat[2, 3]

    return vector_unhat

def Ad(T):                                 # lecture  4, page 11
    R = T[0:3, 0:3]
    p = T[0:3, 3]
    p_hat = hat(p)
    
    adj_T = np.zeros((6, 6))
    adj_T[0:3, 0:3] = R
    adj_T[3:, 0:3] = p_hat @ R
    adj_T[3:, 3:] = R

    return adj_T

def ad(V):                                 # lecture  9, page 16
    w = V[0:3].reshape(3)
    v = V[3:6].reshape(3)

    w_hat = hat(w)
    v_hat = hat(v)
    
    adj_V = np.zeros((6, 6))
    adj_V[0:3, 0:3] = w_hat
    adj_V[3:, 0:3] = v_hat
    adj_V[3:, 3:] = w_hat

    return adj_V

def exp_rotation(theta):                   # lecture  3, page 14
    theta = theta.reshape(3, 1) # θ (bold)
    phi = np.linalg.norm(theta) # θ
    
    R = np.eye(3)

    if not (phi < 1e-12):
        r = theta / phi
        R = np.eye(3) * np.cos(phi) + (1. - np.cos(phi)) * r @ r.T + np.sin(phi) * hat(r.reshape(3))
    
    return R

def log_rotation(R):                       # lecture  3, page 14
    if(np.array_equal(R, np.eye(3))):
        theta = np.zeros((3, 1))
    elif(np.trace(R) == -1):
        phi = np.pi
        theta = phi * ((1. / math.sqrt(2. * (1. + R[0][0]))) * np.array([[1. + R[0][0]], [R[1][0]], [R[2][0]]]))
    else:
        phi = np.arccos((np.trace(R) - 1.) / 2.)
        if (phi < 1e-12):
            theta = np.zeros((3, 1))
        else:
            r_hat = (1. / (2. * np.sin(phi))) * (R - R.T)
            theta = unhat(phi * r_hat)

    return theta

def exp_pose(tau):                         # lecture  4, page 16
    theta = np.linalg.norm(tau[:3, :])
    
    R = np.eye(3)
    A = tau[3:, :]
    
    if not (theta < 1e-12):
        R = exp_rotation(tau[:3, :])
        r = tau[:3, :] / theta
        rho = tau[3:, :] / theta
        r_hat = hat(r.reshape(3))
        A = (np.eye(3) * theta + (1. - np.cos(theta)) * r_hat + (theta - np.sin(theta)) * (r_hat @ r_hat)) @ rho    
    
    T = np.zeros((4, 4))
    T[0:3, 0:3] = R
    T[0:3, 3:] = A
    T[3, :] = [0.0, 0.0, 0.0, 1.0]

    return T

def log_pose(T):                           # lecture  4, page 17
    R = T[:3, :3]
    p = T[:3, 3:]
    
    theta = log_rotation(R)     # θ bold
    phi = np.linalg.norm(theta) # θ

    tau = np.zeros((6, 1))
    tau[3:, :] = p
    
    if not (phi < 1e-12) or (phi == np.pi):
        r_hat = hat(theta.reshape(3)) / phi
        rho = ((1. / phi) * np.eye(3) - r_hat / 2. + (1. / phi - 0.5 / np.tan(phi / 2.)) * (r_hat @ r_hat)) @ p * phi
        
        tau[0:3, :] = theta
        tau[3:, :] = rho

    return tau

def pseudo_inverse(J):                     # lecture  7, page  8
    lamda = 1e-3
    pseudo_inv_J = J.T @ (np.linalg.inv(J @ J.T + lamda**2 * np.eye(6))) # n (= 7) > m (= 6)
    
    return pseudo_inv_J

def J(q):                                  # lecture  6, page 10
    Jw_T = []  # Transpose Space Jacobian (Jw_T) ---> dimensions: (7 x 6)
    exp_p = [] # Exponential Pose

    for i in range(len(q)):
        exp_p.append(exp_pose((Si[i] * q[i]).reshape(6,1)))

    Jw_T.append(Si[0, :])
    Jw_T.append(Ad(exp_p[0]) @ (Si[1, :]))

    x = exp_p[0]
    for i in range(1, len(exp_p)-1):
        y = exp_p[i]
        x = x @ y
        z = Ad(x) @ Si[i+1, :]

        Jw_T.append(z)

    Jw_T = np.array(Jw_T) # dimensions: (7 x 6)
    Jw = Jw_T.T.copy()    # dimensions: (6 x 7)
    
    return Jw

def export_transf_from_par_to_chil(robot): # Transformation Matrix from parent link to child link
    transf_from_par_to_chil = []
    for joint in robot.joints:
        transf_from_par_to_chil.append(joint.origin[:3]) 

    transf_from_par_to_chil = np.array(transf_from_par_to_chil)

    return transf_from_par_to_chil

def export_masses(robot):                  # Masses
    masses = []
    for link in robot.links:
        if link.inertial:
            masses.append(link.inertial.mass)

    masses = np.array(masses)

    return masses

def export_inertia_matrices(robot):        # Rotational Inertia Matrices wrt CoM_i frame (Ib_i)
    I = []
    for link in robot.links:
        if link.inertial:
            I.append(link.inertial.inertia)

    I = np.array(I)

    return I

def body_frames(robot):                    # Mi
    M = np.zeros((6, 4, 4))
    M[0] = Ti_CoMi[0]
    M[1] = T(np.eye(3), [0., 0., 0.0545])
    
    for i in range(2, M.shape[0]):
        M[i] = M[i-1] @ Ti_CoMi[i-1] @ Ti_CoMi[i-1]

    return M

def Mi_i_minus_1():                        # lecture 10, page 12
    M = np.zeros((6, 4, 4))
    Mbase = Mi[0]
    M[0] = np.linalg.inv(Mi[0]) @ Mbase

    for i in range(1, M.shape[0]):
        M[i] = np.linalg.inv(Mi[i]) @ Mi[i-1]

    return M

def Ti_i_minus_1(q):                       # lecture 10, page 12
    T = np.zeros((5, 4, 4))
    for i in range(T.shape[0]-1):
        tau = - Ai[i].reshape(6,1) * q[i]
        T[i] = exp_pose(tau) @ Mij[i+1]
    
    T[4] = np.array([[1., 0., 0.,      0.],
                     [0., 1., 0.,      0.],
                     [0., 0., 1., -0.1605],
                     [0., 0., 0.,      1.]])

    return T

def screw_axes_S(robot):                   # lecture  4, page 14
    # Define r E R(4x3)
    r = []
    for joint in robot.joints:
        r.append(joint.axis) 

    r = np.array(r[:4])

    # Define q_i E R(4x3)
    q = []
    for joint in robot.joints:
        q.append(joint.origin[:3, 3])

    q = np.array(q[:4])
    q[1] += q[0]
    q[2] += q[1]
    q[3] += q[2]

    # Define S E R(4x6)
    S = np.zeros((4, 6))

    for i in range(S.shape[0]):
        r_i = r[i]
        q_i = q[i]
        p_i = -np.cross(r_i, q_i)
        S[i, :3] = r_i
        S[i, 3:] = p_i

    return S

def screw_axes_A():                        # lecture 10, page 12
    A = np.zeros((DoF,6))
    for i in range(A.shape[0]):
        A[i] = Ad(np.linalg.inv(Mi[i+1])) @ Si[i] 
   
    return A

def spatial_inertia_matrix_wrt_CoM(Ib):    # lecture  9, page 16
    G = np.zeros((DoF, 6, 6))
    for i in range (G.shape[0]):
        G[i][:3, :3] = Ib[i]
        G[i][3:, 3:] = m[i] * np.eye(3)

    return G

def spatial_inertia_matrix_wrt_i(Gb):      # lecture  9, page 18
    G = np.zeros((DoF, 6, 6))
    for i in range(1, DoF+1):
        G[i-1] = Ad(np.linalg.inv(Ti_CoMi[i])).T @ Gb[i-1] @ Ad(np.linalg.inv(Ti_CoMi[i]))
    
    return G

def adV(V):                                # lecture 10, page 17
    O = np.zeros((6, 6))
    adV1 = ad(V[:6])
    adV2 = ad(V[5:12])
    adV3 = ad(V[11:18])
    adV4 = ad(V[7:6*DoF])

    adj = np.block([[adV1,    O,    O,    O],
                    [   O, adV2,    O,    O],
                    [   O,    O, adV3,    O],
                    [   O,    O,    O, adV4]])
    
    return adj

def adAqDot(q_dot):                        # lecture 10, page 17
    O = np.zeros((6, 6))
    adA1_q1 = ad(Ai[0].reshape(6,1) * q_dot[0])
    adA2_q2 = ad(Ai[1].reshape(6,1) * q_dot[1])
    adA3_q3 = ad(Ai[2].reshape(6,1) * q_dot[2])
    adA4_q4 = ad(Ai[3].reshape(6,1) * q_dot[3])
    
    adj = np.block([[adA1_q1,       O,       O,       O],
                    [      O, adA2_q2,       O,       O],
                    [      O,       O, adA3_q3,       O],
                    [      O,       O,       O, adA4_q4]])
    
    return adj

def W(q):                                  # lecture 10, page 17
    T = Ti_i_minus_1(q)

    O = np.zeros((6,6))
    T21 = T[1]
    T32 = T[2]
    T43 = T[3]

    AdT21 = Ad(T21)
    AdT32 = Ad(T32)
    AdT43 = Ad(T43)

    W = np.block([[    O,     O,     O, O],
                    [AdT21,     O,     O, O],
                    [    O, AdT32,     O, O],
                    [    O,     O, AdT43, O]])
    
    return W

def L(q):                                  # lecture 10, page 18
    T = Ti_i_minus_1(q)
    
    I = np.eye(6)
    O = np.zeros((6,6))
    T21 = T[1]
    T32 = T[2]
    T43 = T[3]
    T31 = T32 @ T21
    T42 = T43 @ T32
    T41 = T43 @ T32 @ T21

    AdT21 = Ad(T21)
    AdT31 = Ad(T31)
    AdT41 = Ad(T41)
    AdT32 = Ad(T32)
    AdT42 = Ad(T42)
    AdT43 = Ad(T43)

    L = np.block([[    I,     O,     O, O],
                  [AdT21,     I,     O, O],
                  [AdT31, AdT32,     I, O],
                  [AdT41, AdT42, AdT43, I]])
    return L

def M(q):                                  # lecture 10, page 20
    Lall = L(q)
    M_q = Aall.T @ Lall.T @ Gall @ Lall @ Aall

    return M_q

def C(q, q_dot):                           # lecture 10, page 20
    V0 = np.zeros((6, 1))
    V0_dot = np.array([[0.], [0.], [0.], [0.], [0.], [-9.81]])

    Vbase = np.zeros((6*DoF, 1))
    Vbase[:6] = Ad(Ti_CoMi[0]) @ V0

    Vbase_dot = np.zeros((6*DoF, 1))
    Vbase_dot[:6] = Ad(Ti_CoMi[0]) @ V0_dot

    adAq_dot = adAqDot(q_dot)
    Lall = L(q)
    Wall = W(q)

    V = Lall @ (Aall @ q_dot + Vbase)
    adVall = adV(V)

    c = - Aall.T @ Lall.T @ (Gall @ Lall @ adAq_dot @ Wall + adVall.T @ Gall) @ Lall @ Aall @ q_dot
    g = Aall.T @ Lall.T @ Gall @ Lall @ Vbase_dot
    C_q_q_dot = c + g

    return C_q_q_dot

def FK(q):                                 # lecture  5, page  6
    Tw_ee = np.eye(4)
    for i in range(len(q)):
        Tw_ee = Tw_ee @ exp_pose((Si[i] * q[i]).reshape(6, 1))

    Tw_ee = Tw_ee @ Mi[5]

    return Tw_ee

def IK(T_desired, q_actual):               # lecture  6, page 22
    q = q_actual
    Vw = np.zeros((6, 1)) # Vw (Space Twist)
    
    # Newton-Raphson Algorithm
    while((np.linalg.norm(Vw) >= 1e-3 or np.linalg.norm(Vw) == 0)):
        current_pos = np.linalg.inv(FK(q))
        Vw = log_pose(T_desired @ current_pos)
        Jw_pseudo_inv = pseudo_inverse(J(q))
        dq = Jw_pseudo_inv @ Vw
        q += dq

    return q

def FD(q, q_dot, tau):                     # lecture 11, page  3
    q_2dot = np.linalg.inv(M(q)) @ (tau - C(q, q_dot)) # Forward Dynamics

    return q_2dot

def ID(q_desired, q_desired_dot, q_desired_2dot, q_actual, q_actual_dot): # lecture 11, page  9
    Kp = 390.
    Kd = 2 * np.sqrt(Kp)
    qe = q_desired - q_actual
    deriv_qe = q_desired_dot - q_actual_dot 

    tau = M(q_actual) @ (q_desired_2dot + Kp * qe + Kd * deriv_qe) + C(q_desired, q_desired_dot)
    
    return tau

def step(q, q_dot, q_2dot):
    dt = TIME_STEP
    q_new = q + q_dot * dt
    q_dot_new = q_dot + q_2dot * dt

    # Limits
    limits = True
    max_v = [5., 5., 5., 5.]

    for i in range(DoF):
        if limits:
            q_dot_new[i] = min(max(q_dot_new[i], -max_v[i]), max_v[i])

    return q_new, q_dot_new

def T_cubic_splines(t):                    # lecture  8, page  9
    tau = unhat(hat(c3) * (t**3) + hat(c2) * (t**2) + hat(c1) * t + hat(c0))
    T = exp_pose(tau)

    return T

def T_dot_cubic_splines(t):                # lecture  8, page  9
    T_dot = (3 * hat(c3) * (t**2) + 2 * hat(c2) * t + hat(c1)) @ T_cubic_splines(t) 

    return T_dot

def trajectory(Ts, Tg):                    # lecture  8, page  9
    tau_s = log_pose(Ts) # τs
    tau_g = log_pose(Tg) # τg

    Vs = np.array([[0.], [0.], [0.], [0.], [0.], [0.]]).reshape(6, 1) # starting velocities
    Vg = np.array([[0.], [0.], [0.], [0.], [0.], [0.]]).reshape(6, 1) #  desired velocities

    # Coefficients
    global c0, c1, c2, c3

    c0 = tau_s.reshape(6)
    c1 = Vs.reshape(6)
    c2 = (3 * tau_g / (TOTAL_TIME**2) - 3 * tau_s / (TOTAL_TIME**2) - 2 * Vs / TOTAL_TIME - Vg / TOTAL_TIME).reshape(6)
    c3 = (-2 * tau_g / (TOTAL_TIME**3) + 2 * tau_s / (TOTAL_TIME**3) + Vs / (TOTAL_TIME**2) + Vg / (TOTAL_TIME**2)).reshape(6)

    # Trajectory
    times = np.arange(0., TOTAL_TIME + TIME_STEP, TIME_STEP)
    traj = []

    for t in times:
        traj.append(T_cubic_splines(t))
        
    traj = np.array(traj)

    return traj

def desired_commands():
    # Trajectory
    Ts1 = np.array([[1., 0., 0.,    0.],
                    [0., 1., 0.,    0.],
                    [0., 0., 1., 0.857],
                    [0., 0., 0.,    1.]])
    Tg1 = np.array([[-0.2957, 0.,  0.9553, 0.2958],
                    [     0., 1.,      0.,     0.],
                    [-0.9553, 0., -0.2957, 0.6187],
                    [     0., 0.,      0.,     1.]])
    Ts2 = Tg1
    Tg2 = np.array([[-0.8378, 0.,  0.5460, 0.3178],
                    [     0., 1.,      0.,     0.],
                    [-0.5460, 0., -0.8378, 0.4261],
                    [     0., 0.,      0.,     1.]])
    Ts3 = Tg2
    Tg3 = np.array([[ 0.7235, 0., 0.6904, 0.4712],
                    [     0., 1.,     0.,     0.],
                    [-0.6904, 0., 0.7235, 0.6187],
                    [     0., 0.,     0.,     1.]])
    Ts4 = Tg3
    Tg4 = np.array([[ 0.7912, 0., 0.6115, 0.3831],
                    [     0., 1.,     0.,     0.],
                    [-0.6115, 0., 0.7912, 0.7354],
                    [     0., 0.,     0.,     1.]])
    
    traj1 = trajectory(Ts1, Tg1)
    traj2 = trajectory(Ts2, Tg2)
    traj3 = trajectory(Ts3, Tg3)
    traj4 = trajectory(Ts4, Tg4)
    traj = np.vstack((traj1, traj2, traj3, traj4))
    
    dt = TIME_STEP

    # Position
    q_desired = np.zeros((traj.shape[0], DoF, 1))
    q_desired[0] = np.array([[0.00001], [0.00001], [0.00001], [0.00001]])
    for i in range((traj.shape[0] - 1)):
        q_desired[i + 1] = IK(traj[i + 1], q_desired[i].copy())

    # Velocity
    q_desired_dot = np.zeros((traj.shape[0], DoF, 1)) 
    for i in range(traj.shape[0]-1):
        q_desired_dot[i] = (q_desired[i + 1] - q_desired[i]) / dt

    # Acceleration
    q_desired_2dot = np.zeros((traj.shape[0], DoF, 1)) 
    for i in range(traj.shape[0]-1):
        q_desired_2dot[i] = (q_desired_dot[i + 1] - q_desired_dot[i]) / dt

    return q_desired, q_desired_dot, q_desired_2dot

def formatted_data(q, file_index):
    formatted_data = []
    i = 0
    for row in q:
        if i == 0:
            formatted_row = "[[" + ", ".join(f"{x:.4f}" for x in row) + "]" + ","
        elif i == len(q) - 1:
            formatted_row = "[" + ", ".join(f"{x:.4f}" for x in row) + "]]"
        else:
            formatted_row = "[" + ", ".join(f"{x:.4f}" for x in row) + "]" + ","
        
        formatted_data.append(formatted_row)
        i += 1

    output = "\n".join(formatted_data)

    file_name = f"q{file_index}.txt"

    with open(file_name, "w") as file:
        file.write(output)

    return 0

def main():
    global Ti_CoMi, m, Ib, Gb, Gi, Mi, Mij, Si, Ai, Aall, Gall

    Ti_CoMi = []
    for link in robot.links:
        Ti_CoMi.append(link.inertial.origin)

    Ti_CoMi = np.array(Ti_CoMi)

    # Masses of links
    m = export_masses(robot)[1:5]  # mi (link 1 - link 4)

    # Rotational Intertia Matrices Ib_i wrt CoM frame (link 1 - link 4)
    Ib = export_inertia_matrices(robot)[1:5]

    # Spatial Inertia Matrices Gb_i wrt CoM frame (link 1 - link 4)
    Gb = spatial_inertia_matrix_wrt_CoM(Ib)
    
    # Spatial Inertia Matrices Gi wrt link frame (link 1 - link 4)
    Gi = spatial_inertia_matrix_wrt_i(Gb)

    Mi = body_frames(robot)
    Mij = Mi_i_minus_1()
    Si = screw_axes_S(robot)
    Ai = screw_axes_A()

    Aall = np.block([[Ai[0].reshape(6,1),   np.zeros((6, 1)),   np.zeros((6, 1)),   np.zeros((6, 1))],
                     [  np.zeros((6, 1)), Ai[1].reshape(6,1),   np.zeros((6, 1)),   np.zeros((6, 1))],
                     [  np.zeros((6, 1)),   np.zeros((6, 1)), Ai[2].reshape(6,1),   np.zeros((6, 1))],
                     [  np.zeros((6, 1)),   np.zeros((6, 1)),   np.zeros((6, 1)), Ai[3].reshape(6,1)]])

    Gall = np.block([[           Gi[0], np.zeros((6, 6)), np.zeros((6, 6)), np.zeros((6, 6))],
                     [np.zeros((6, 6)),            Gi[1], np.zeros((6, 6)), np.zeros((6, 6))],
                     [np.zeros((6, 6)), np.zeros((6, 6)),            Gi[2], np.zeros((6, 6))],
                     [np.zeros((6, 6)), np.zeros((6, 6)), np.zeros((6, 6)),            Gi[3]]])

    positions = []

    # Initializations
    q_desired, q_desired_dot, q_desired_2dot = desired_commands()
    q_actual = np.array([[0.], [0.], [0.], [0.]])
    q_actual_dot = np.array([[0.], [0.], [0.], [0.]])
    tau = np.array([[0.], [0.], [0.], [0.]])
    
    for i in range(q_desired.shape[0]):
        positions.append(normalize(q_actual))

        # Forward Dynamics
        q_actual_2dot = FD(q_actual, q_actual_dot, tau)
        
        # Inverse Dynamics Controller
        tau = ID(q_desired[i], q_desired_dot[i], q_desired_2dot[i], q_actual, q_actual_dot)

        # Euler Integration
        q_actual, q_actual_dot = step(q_actual, q_actual_dot, q_actual_2dot)

    positions = np.array(positions).reshape(int(q_desired.shape[0]), DoF)
    formatted_data(positions, 1)
    
if __name__ == "__main__":
    main()
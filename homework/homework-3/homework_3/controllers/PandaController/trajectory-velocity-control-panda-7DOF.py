import numpy as np
import math
import matplotlib.pyplot as plt

np.set_printoptions(precision=2, suppress=True, floatmode='fixed')

PI = np.pi
TOTAL_TIME = 100. # 100 sec ---> Webots: 100000 ms
TIME_STEP = 0.1   # 0.1 sec ---> Webots:    100 ms

# Screw Axis
r = np.array([
    [0.0, 0.0, 1.0],  # 1
    [0.0, 1.0, 0.0],  # 2
    [0.0, 0.0, 1.0],  # 3
    [0.0, -1.0, 0.0], # 4
    [0.0, 0.0, 1.0],  # 5
    [0.0, -1.0, 0.0], # 6
    [0.0, 0.0, -1.0]  # 7
])

q = np.array([
    [0.0, 0.0, 0.0],
    [0.0, 0.0, 0.333],
    [0.0, 0.0, 0.0],
    [0.088, 0.0, 0.649],
    [0.0, 0.0, 0.0],
    [0.0, 0.0, 1.033],
    [0.088, 0.0, 0.0]
])

S = np.zeros((7, 6))

for i in range(S.shape[0]):
    r_i = r[i]
    q_i = q[i]
    p_i = -np.cross(r_i, q_i)
    S[i, :3] = r_i
    S[i, 3:] = p_i

def Rx(theta):                      # lecture 2, page 11
    theta = theta * (PI / 180.0) # degrees to rad
    
    R = np.array([
        [1.0,           0.0,            0.0],
        [0.0, np.cos(theta), -np.sin(theta)],
        [0.0, np.sin(theta),  np.cos(theta)]
    ])
    
    return R 

def Ry(theta):                      # lecture 2, page 11
    theta = theta * (PI / 180.0) # degrees to rad
    
    R = np.array([
        [ np.cos(theta), 0.0, np.sin(theta)],
        [           0.0, 1.0,           0.0],
        [-np.sin(theta), 0.0, np.cos(theta)]
    ])
    
    return R

def Rz(theta):                      # lecture 2, page 11
    theta = theta * (PI / 180.0) # degrees to rad
    
    R = np.array([
        [np.cos(theta), -np.sin(theta), 0.0],
        [np.sin(theta),  np.cos(theta), 0.0],
        [          0.0,            0.0, 1.0]
    ])
    
    return R

def T(R, p):                        # lecture 2, page 13
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    
    return T

def hat(vector):                    # lecture 3, page  5 - lecture 4, page 14
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

def unhat(vector_hat):              # lecture 3, page  5 - lecture 4, page 14
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

def Ad(T):                          # lecture 4, page 11
    R = T[0:3, 0:3]
    p = T[0:3, 3]

    p_hat = hat(p)
    
    adj_T = np.zeros((6, 6))
    adj_T[0:3, 0:3] = R
    adj_T[3:, 0:3] = p_hat @ R
    adj_T[3:, 3:] = R

    return adj_T

def exp_rotation(theta):            # lecture 3, page 14
    theta = theta.reshape(3, 1) # θ (bold)
    phi = np.linalg.norm(theta) # θ
    
    R = np.eye(3)

    if not (phi < 1e-12):
        r = theta / phi
        R = np.eye(3) * np.cos(phi) + (1. - np.cos(phi)) * r @ r.T + np.sin(phi) * hat(r.reshape(3))
    
    return R

def log_rotation(R):                # lecture 3, page 14
    if(np.array_equal(R, np.eye(3))):
        theta = np.zeros((3, 1))
    elif(np.trace(R) == -1):
        phi = PI
        theta = phi * ((1. / math.sqrt(2. * (1. + R[0][0]))) * np.array([[1. + R[0][0]], [R[1][0]], [R[2][0]]]))
    else:
        phi = np.arccos((np.trace(R) - 1.) / 2.)
        if (phi < 1e-12):
            theta = np.zeros((3, 1))
        else:
            r_hat = (1. / (2. * np.sin(phi))) * (R - R.T)
            theta = unhat(phi * r_hat)

    return theta

def exp_pose(tau):                  # lecture 4, page 16
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

def log_pose(T):                    # lecture 4, page 17
    R = T[:3, :3]
    p = T[:3, 3:]
    
    theta = log_rotation(R)     # θ bold
    phi = np.linalg.norm(theta) # θ

    tau = np.zeros((6, 1))
    tau[3:, :] = p
    
    if not (phi < 1e-12) or (phi == PI):
        r_hat = hat(theta.reshape(3)) / phi
        rho = ((1. / phi) * np.eye(3) - r_hat / 2. + (1. / phi - 0.5 / np.tan(phi / 2.)) * (r_hat @ r_hat)) @ p * phi
        
        tau[0:3, :] = theta
        tau[3:, :] = rho

    return tau

def FK(S, theta, M):                # lecture 5, page  6
    Tw_ee = np.eye(4)

    for i in range(len(theta)):        
        Tw_ee = Tw_ee @ exp_pose((S[i] * theta[i]).reshape(6, 1))

    Tw_ee = Tw_ee @ M

    return Tw_ee

def pseudo_inverse(J):              # lecture 7, page  8
    lamda = 0.1
    pseudo_inv_J = J.T @ (np.linalg.inv(J @ J.T + lamda**2 * np.eye(6))) # n (= 7) > m (= 6)        

    return pseudo_inv_J

def T_cubic_splines(t):             # lecture 8, page  9
    tau = unhat(hat(c3) * (t**3) + hat(c2) * (t**2) + hat(c1) * t + hat(c0))
    T = exp_pose(tau)

    return T

def T_dot_cubic_splines(t):         # lecture 8, page  9
    T_dot = (3 * hat(c3) * (t**2) + 2 * hat(c2) * t + hat(c1)) @ T_cubic_splines(t) 

    return T_dot

def trajectory(Ts, Tg, Vs, Vg):     # lecture 8, page  9
    tau_s = log_pose(Ts) # τs
    tau_g = log_pose(Tg) # τg

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
    
    # Plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection = '3d')

    # World frame
    origin = [0., 0., 0.]
    x_axis = [1., 0., 0.]
    y_axis = [0., 1., 0.]
    z_axis = [0., 0., 1.]

    ax.quiver(*origin, *x_axis, color='m', length=0.5, normalize=True)
    ax.quiver(*origin, *y_axis, color='y', length=0.5, normalize=True)
    ax.quiver(*origin, *z_axis, color='k', length=0.5, normalize=True)

    # Trajectory frames
    for i in range(len(traj)):
        origin = traj[i][:3, 3]
        x_axis = traj[i][:3, 0]
        y_axis = traj[i][:3, 1]
        z_axis = traj[i][:3, 2]

        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)
        z_axis = z_axis / np.linalg.norm(z_axis)

        ax.quiver(*origin, *x_axis, color='r', length=0.5, normalize=True)
        ax.quiver(*origin, *y_axis, color='g', length=0.5, normalize=True)
        ax.quiver(*origin, *z_axis, color='b', length=0.5, normalize=True)

    ax.set_xlim([0., 2.5])
    ax.set_ylim([0., 2.5])
    ax.set_zlim([0., 2.5])

    plt.show()

    return traj

def control(trajectory, initial_q): # lecture 7, page  7
    limits = True
    max_v = [2.5, 2.5, 2.5, 2.5, 3., 3., 3.]

    # PI-controller
    Kp = 5.
    Ki = 0.01
    dt = TIME_STEP
    integralXe = 0.
        
    v = []

    # Desired space
    q = initial_q
    
    times = np.arange(0., TOTAL_TIME+TIME_STEP, TIME_STEP)
    i = 0
    for t in times:
        current_pos = np.linalg.inv(FK(S, q, M))         # Twb^{-1}
        goal_pos = trajectory[i]                         # Twd

        Xe = log_pose(current_pos @ goal_pos).reshape(6) # Xe = log(Twb^{-1} @ Twd)
        integralXe += Xe * dt

        Vd = unhat(np.linalg.inv(T_cubic_splines(t)) @ T_dot_cubic_splines(t))
        Vb = (Ad(current_pos @ goal_pos) @ Vd).reshape(6) + Kp * Xe + Ki * integralXe

        # Jacobian
        Jw_T = []  # Transpose Space Jacobian (Jw_T) ---> dimensions: (7 x 6)
        exp_p = [] # Exponential Pose

        for j in range(len(q)):
            exp_p.append(exp_pose((S[j] * q[j]).reshape(6, 1)))

        Jw_T.append(S[0, :])
        Jw_T.append(Ad(exp_p[0]) @ (S[1, :]))

        x = exp_p[0]
        for j in range(1, len(exp_p)-1):
            y = exp_p[j]
            x = x @ y
            z = Ad(x) @ S[j+1, :]

            Jw_T.append(z)

        Jw_T = np.array(Jw_T) # dim.: (7 x 6)
        Jw = Jw_T.T.copy()    # dim.: (6 x 7)
        
        Jb = Ad(current_pos) @ Jw
        Jb_pseudo_inv = pseudo_inverse(Jb)

        v.append(Jb_pseudo_inv @ Vb)

        q = q + v[i] * dt
        i += 1

    v = np.array(v).reshape(len(v),7)

    # Limits
    for i in range(int(TOTAL_TIME / dt) + 1):
        for j in range(7):
            if limits:
                v[i][j] = min(max(v[i][j], -max_v[j]), max_v[j])

    return v, q

def formatted_data(v, file_index):
    formatted_data = []
    i = 0
    for row in v:
        if i == 0:
            formatted_row = "[[" + ", ".join(f"{x:.4f}" for x in row) + "]" + ","
        elif i == len(v) - 1:
            formatted_row = "[" + ", ".join(f"{x:.4f}" for x in row) + "]]"
        else:
            formatted_row = "[" + ", ".join(f"{x:.4f}" for x in row) + "]" + ","
        
        formatted_data.append(formatted_row)
        i += 1

    output = "\n".join(formatted_data)

    file_name = f"v{file_index}.txt"

    with open(file_name, "w") as file:
        file.write(output)

    return 0

def main():
    global Rw, M, T_desired_1, T_desired_2

    Rw = np.eye(3)
    Rw_base = Rw
    Rw_xF = np.dot(Rw_base, np.dot(np.dot(Rx(180.0), Ry(0.0)), Rz(0.0)))
    
    Rw_ee = np.dot(Rw_xF, np.dot(np.dot(Rx(0.0), Ry(0.0)), Rz(-45.0)))
    pw_ee = [0.088, 0.0, 0.823]

    Rw_ee_desired_1 = np.dot(np.dot(np.dot(Rx(0.0), Ry(0.0)), Rz(-45.0)), Rw_ee)
    pw_ee_desired_1 = [0.695, 0.095, 0.025]

    Rw_ee_desired_2 = np.dot(np.dot(np.dot(Rx(0.0), Ry(-90.0)), Rz(-45.0)), Rw_ee)
    pw_ee_desired_2 = [0.5, 0.55, 0.45]
    
    # Configuration of the end-effector while at home configuration
    M = T(Rw_ee, pw_ee)

    # Desired configuration of the end-effector (above the cylinder)
    T_desired_1 = T(Rw_ee_desired_1, pw_ee_desired_1)

    # Desired configuration of the end-effector (above the hole)
    T_desired_2 = T(Rw_ee_desired_2, pw_ee_desired_2) 
    
    # Intermediate configurations
    T_intermediate_1 = T(Rw_ee_desired_1, [0.77, 0.095, 0.77])
    T_intermediate_2 = T(Rw_ee_desired_2, [-0.23, 0.5, 0.978])
    
    # ------------------------------------------------------------------------------------- #
    
    initial_q = [0., 0., 0., -0.000001, 0., 0., -0.000001]
    
    Ts_1 = M                # starting configuration ---> home configuration
    Tg_1 = T_intermediate_1 #  desired configuration ---> intermediate configuration (1)

    Vs_1 = np.array([[0.], [0.], [0.], [0.], [0.], [0.]]).reshape(6, 1) # starting velocities
    Vg_1 = np.array([[0.], [0.], [0.], [0.], [0.], [0.]]).reshape(6, 1) #  desired velocities

    # Trajectory: home configuration ---> intermediate configuration (1)
    trajectory_1 = trajectory(Ts_1, Tg_1, Vs_1, Vg_1)  # trajectory
    v1, q1 = control(trajectory_1, initial_q)          # joint angles of the last frame
    formatted_data(v1, 1)

    # ------------------------------------------------------------------------------------- #
        
    Ts_2 = T_intermediate_1 # starting configuration ---> intermediate configuration (1)
    Tg_2 = T_desired_1      #  desired configuration ---> above the cylinder

    Vs_2 = np.array([[0.], [0.], [0.], [0.], [0.], [0.]]).reshape(6, 1) # starting velocities
    Vg_2 = np.array([[0.], [0.], [0.], [0.], [0.], [0.]]).reshape(6, 1) #  desired velocities

    # Trajectory: intermediate configuration (1) ---> above the cylinder
    trajectory_2 = trajectory(Ts_2, Tg_2, Vs_2, Vg_2) # trajectory
    v2, q2 = control(trajectory_2, q1)                # joint angles of the last frame
    formatted_data(v2, 2)

    # ------------------------------------------------------------------------------------- #

    Ts_3 = T_desired_1      # starting configuration ---> above the cylinder
    Tg_3 = T_intermediate_1 #  desired configuration ---> intermediate configuration (1)

    Vs_3 = np.array([[0.], [0.], [0.], [0.], [0.], [0.]]).reshape(6, 1) # starting velocities
    Vg_3 = np.array([[0.], [0.], [0.], [0.], [0.], [0.]]).reshape(6, 1) #  desired velocities

    # Trajectory: above the cylinder ---> intermediate configuration (1)
    trajectory_3 = trajectory(Ts_3, Tg_3, Vs_3, Vg_3) # trajectory
    v3, q3 = control(trajectory_3, q2)                # joint angles of the last frame
    formatted_data(v3, 3)

    # ------------------------------------------------------------------------------------- #

    Ts_4 = T_intermediate_1 # starting configuration ---> intermediate configuration (1)
    Tg_4 = T_intermediate_2 #  desired configuration ---> intermediate configuration (2)

    Vs_4 = np.array([[0.], [0.], [0.], [0.], [0.], [0.]]).reshape(6, 1) # starting velocities
    Vg_4 = np.array([[0.], [0.], [0.], [0.], [0.], [0.]]).reshape(6, 1) #  desired velocities

    # Trajectory: intermediate configuration (1) ---> intermediate configuration (2)
    trajectory_4 = trajectory(Ts_4, Tg_4, Vs_4, Vg_4) # trajectory
    v4, q4 = control(trajectory_4, q3)                # joint angles of the last frame
    formatted_data(v4, 4)

    # ------------------------------------------------------------------------------------- #

    Ts_5 = T_intermediate_2 # starting configuration ---> intermediate configuration (2)
    Tg_5 = T_desired_2      #  desired configuration ---> above the hole

    Vs_5 = np.array([[0.], [0.], [0.], [0.], [0.], [0.]]).reshape(6, 1) # starting velocities
    Vg_5 = np.array([[0.], [0.], [0.], [0.], [0.], [0.]]).reshape(6, 1) #  desired velocities

    # Trajectory: intermediate configuration (2) ---> above the hole
    trajectory_5 = trajectory(Ts_5, Tg_5, Vs_5, Vg_5) # trajectory
    v5, q5 = control(trajectory_5, q4)                # joint angles of the last frame
    formatted_data(v5, 5)

if __name__ == "__main__":
    main()
import numpy as np
import matplotlib.pyplot as plt

def ZUPTaidedINS(u, zupt, logL, adpt_flag, simdata):
    N = len(u[0])
    P, Q, R, H = init_filter(simdata) # checked

    x_h, cov, Id = init_vec(N, P, simdata) #checked
    x_h[0:9, 0], quat = init_Nav_eq(u, simdata) # checked function not output
    # checked, replace the first row of x_h (transposed in matlab framwork) 
    if adpt_flag:
        zupt = np.zeros((1,len(logL[0]) + 5), dtype=bool)
        delta_t = 0
        gamma = np.zeros(len(logL[0]))
        if simdata['detector_prio'] == 'normalized velocity':
            c1, c2, c3 = -1e2 * simdata['Window_size'] / 2, -5e4 * simdata['Window_size'] / 2, 100
        else:
            c1, c2, c3 = -1e2 * simdata['Window_size'] / 2, -5e4 * simdata['Window_size'] / 2, 0

    for k in range(1, N):
        u_h = comp_imu_errors(u[:, k], x_h[:, k-1], simdata) # checked
        x_h[:, k], quat = Navigation_equations(x_h[:, k-1], u_h, quat, simdata) # checked
        F, G = state_matrix(quat, u_h, simdata) # checked

        P = F @ P @ F.T + G @ Q @ G.T
        P = (P + P.T) / 2
        cov[:, k] = np.diag(P)

        if adpt_flag:
            S = P[3:6, 3:6]
            v = x_h[3:6, k]
  
            gamma[k] = c1 + c2 * delta_t + c3 * (v.T @ np.linalg.inv(S) @ v)
            if logL[0][k] > gamma[k]:
                zupt[0][k:k + simdata['Window_size']] = True
                delta_t = 0
            else:
                delta_t += simdata['Ts']

        if zupt[0][k]:
            K = P @ H.T @ np.linalg.inv(H @ P @ H.T + R)
            z = -x_h[3:6, k]
            dx = K @ z
            x_h[:, k], quat = comp_internal_states(x_h[:, k], dx, quat)
            P = (Id - K @ H) @ P
            P = (P + P.T) / 2
            cov[:, k] = np.diag(P)

    if adpt_flag:
        t = simdata['Ts'] * np.arange(N)
        gamma[gamma > -1e2] = -1e1
        plt.figure()
        plt.clf()
        plt.semilogy(t, logL.ravel(), 'k')
        plt.semilogy(t, gamma, 'r')
        plt.yscale('symlog')
        plt.grid(which='minor')
        plt.ylim([-1e8, -1e2])
        plt.show()

    return x_h, cov

def init_vec(N, P, simdata): # checked
    if simdata['scalefactors'] == 'on' and simdata['biases'] == 'on':
        cov = np.zeros((9 + 6 + 6, N))
        x_h = np.zeros((9 + 6 + 6, N))
    elif simdata['scalefactors'] == 'on' and simdata['biases'] == 'off':
        cov = np.zeros((9 + 6, N))
        x_h = np.zeros((9 + 6, N))
    elif simdata['scalefactors'] == 'off' and simdata['biases'] == 'on':
        cov = np.zeros((9 + 6, N))
        x_h = np.zeros((9 + 6, N))
    else:
        cov = np.zeros((9, N))
        x_h = np.zeros((9, N))
    Id = np.eye(P.shape[0])
    cov[:, 0] = np.diag(P)
    return x_h, cov, Id

def init_Nav_eq(u, simdata): # checked
    f_u = np.mean(u[0, :20])
    f_v = np.mean(u[1, :20])
    f_w = np.mean(u[2, :20])

    roll = np.arctan2(-f_v, -f_w)
    pitch = np.arctan2(f_u, np.sqrt(f_v ** 2 + f_w ** 2))

    attitude = np.array([roll, pitch, simdata['init_heading']])
    Rb2t = Rt2b(attitude).T
    quat = dcm2q(Rb2t)

    x = np.zeros(9)
    x[0:3] = simdata['init_pos'].flatten()
    x[6:9] = attitude
    return x, quat

def init_filter(simdata): # checked
    if simdata['scalefactors'] == 'on' and simdata['biases'] == 'on':
        P = np.zeros((9 + 6 + 6, 9 + 6 + 6))
        P[9:12, 9:12] = np.diag(simdata['sigma_initial_acc_bias'].ravel() ** 2)
        P[12:15, 12:15] = np.diag(simdata['sigma_initial_gyro_bias'].ravel() ** 2)
        P[15:18, 15:18] = np.diag(simdata['sigma_initial_acc_scale'].ravel() ** 2)
        P[18:21, 18:21] = np.diag(simdata['sigma_initial_gyro_scale'].ravel() ** 2)
        Q = np.zeros((12, 12))
        Q[6:9, 6:9] = np.diag(simdata['acc_bias_driving_noise'].ravel() ** 2)
        Q[9:12, 9:12] = np.diag(simdata['gyro_bias_driving_noise'].ravel() ** 2)
        H = np.zeros((3, 9 + 6 + 6))
    elif simdata['scalefactors'] == 'on' and simdata['biases'] == 'off':
        P = np.zeros((9 + 6, 9 + 6))
        P[9:12, 9:12] = np.diag(simdata['sigma_initial_acc_scale'].ravel() ** 2)
        P[12:15, 12:15] = np.diag(simdata['sigma_initial_gyro_scale'].ravel() ** 2)
        Q = np.zeros((6, 6))
        H = np.zeros((3, 9 + 6))
    elif simdata['scalefactors'] == 'off' and simdata['biases'] == 'on':
        P = np.zeros((9 + 6, 9 + 6))
        P[9:12, 9:12] = np.diag(simdata['sigma_initial_acc_bias'].ravel() ** 2)
        P[12:15, 12:15] = np.diag(simdata['sigma_initial_gyro_bias'].ravel() ** 2)
        Q = np.zeros((12, 12))
        Q[6:9, 6:9] = np.diag(simdata['acc_bias_driving_noise'].ravel() ** 2)
        Q[9:12, 9:12] = np.diag(simdata['gyro_bias_driving_noise'].ravel() ** 2)
        H = np.zeros((3, 9 + 6))
    else:
        P = np.zeros((9, 9))
        Q = np.zeros((6, 6))
        H = np.zeros((3, 9))

    H[0:3, 3:6] = np.eye(3)
    P[0:3, 0:3] = np.diag(simdata['sigma_initial_pos'].ravel() ** 2)
    P[3:6, 3:6] = np.diag(simdata['sigma_initial_vel'].ravel() ** 2)
    P[6:9, 6:9] = np.diag(simdata['sigma_initial_att'].ravel() ** 2)
    Q[0:3, 0:3] = np.diag(simdata['sigma_acc'].ravel() ** 2)
    Q[3:6, 3:6] = np.diag(simdata['sigma_gyro'].ravel() ** 2)
    R = np.diag(simdata['sigma_vel'] ** 2)

    return P, Q, R, H

def Navigation_equations(x, u, q, simdata): # checked
    y = np.zeros_like(x)
    Ts = simdata['Ts']

    w_tb = u[3:6]
    P = w_tb[0] * Ts
    Q = w_tb[1] * Ts
    R = w_tb[2] * Ts

    OMEGA = np.array([[0, R, -Q, P], [-R, 0, P, Q], [Q, -P, 0, R], [-P, -Q, -R, 0]]) * 0.5

    v = np.linalg.norm(w_tb) * Ts
    if v != 0:
        q = (np.cos(v / 2) * np.eye(4) + 2 / v * np.sin(v / 2) * OMEGA) @ q
        q = q / np.linalg.norm(q)

    Rb2t = q2dcm(q) # checked
    y[6] = np.arctan2(Rb2t[2, 1], Rb2t[2, 2])
    y[7] = -np.arctan(Rb2t[2, 0] / np.sqrt(1 - Rb2t[2, 0] ** 2))
    y[8] = np.arctan2(Rb2t[1, 0], Rb2t[0, 0])

    g_t = np.array([0, 0, simdata['g']])
    f_t = Rb2t @ u[0:3]
    acc_t = f_t + g_t

    A = np.eye(6)
    A[0, 3] = Ts
    A[1, 4] = Ts
    A[2, 5] = Ts
    B = np.vstack(((Ts ** 2) / 2 * np.eye(3), Ts * np.eye(3)))

    y[0:6] = A @ x[0:6] + B @ acc_t

    return y, q

def state_matrix(q, u, simdata): # checked
    Rb2t = q2dcm(q)
    f_t = Rb2t @ u[0:3]
    St = np.array([[0, -f_t[2], f_t[1]], [f_t[2], 0, -f_t[0]], [-f_t[1], f_t[0], 0]])

    O = np.zeros((3, 3))
    I = np.eye(3)
    Da = np.diag(u[0:3])
    Dg = np.diag(u[3:6])

    B1 = -1 / simdata['acc_bias_instability_time_constant_filter'] * np.eye(3)
    B2 = -1 / simdata['gyro_bias_instability_time_constant_filter'] * np.eye(3)

    if simdata['scalefactors'] == 'on' and simdata['biases'] == 'on':
        Fc = np.block([
            [O, I, O, O, O, O, O],
            [O, O, St, Rb2t, O, Rb2t @ Da, O],
            [O, O, O, O, -Rb2t, O, -Rb2t @ Dg],
            [O, O, O, B1, O, O, O],
            [O, O, O, O, B2, O, O],
            [O, O, O, O, O, O, O],
            [O, O, O, O, O, O, O]
        ])
        Gc = np.block([
            [O, O, O, O],
            [Rb2t, O, O, O],
            [O, -Rb2t, O, O],
            [O, O, I, O],
            [O, O, O, I],
            [O, O, O, O],
            [O, O, O, O]
        ])
    elif simdata['scalefactors'] == 'on' and simdata['biases'] == 'off':
        Fc = np.block([
            [O, I, O, O, O],
            [O, O, St, Rb2t @ Da, O],
            [O, O, O, O, -Rb2t @ Dg],
            [O, O, O, O, O],
            [O, O, O, O, O]
        ])
        Gc = np.block([
            [O, O],
            [Rb2t, O],
            [O, -Rb2t],
            [O, O],
            [O, O]
        ])
    elif simdata['scalefactors'] == 'off' and simdata['biases'] == 'on':
        Fc = np.block([
            [O, I, O, O, O],
            [O, O, St, Rb2t, O],
            [O, O, O, O, -Rb2t],
            [O, O, O, B1, O],
            [O, O, O, O, B2]
        ])
        Gc = np.block([
            [O, O, O, O],
            [Rb2t, O, O, O],
            [O, -Rb2t, O, O],
            [O, O, I, O],
            [O, O, O, I]
        ])
    else:
        Fc = np.block([
            [O, I, O],
            [O, O, St],
            [O, O, O]
        ])
        Gc = np.block([
            [O, O],
            [Rb2t, O],
            [O, -Rb2t]
        ])

    F = np.eye(Fc.shape[0]) + (simdata['Ts'] * Fc)
    G = simdata['Ts'] * Gc
    return F, G

def comp_internal_states(x_in, dx, q_in):
    R = q2dcm(q_in)
    x_out = x_in + dx

    epsilon = dx[6:9]
    # OMEGA = np.array([[0, -epsilon[2], epsilon[1]], [epsilon[2, 0, -epsilon[0]], [-epsilon[1], epsilon[0], 0]]])

    OMEGA = np.array([[0, -epsilon[2], epsilon[1]], 
                  [epsilon[2], 0, -epsilon[0]], 
                  [-epsilon[1], epsilon[0], 0]])
    

    R = (np.eye(3) - OMEGA) @ R

    x_out[6] = np.arctan2(R[2, 1], R[2, 2])
    x_out[7] = -np.arctan(R[2, 0] / np.sqrt(1 - R[2, 0] ** 2))
    x_out[8] = np.arctan2(R[1, 0], R[0, 0])

    q_out = dcm2q(R)
    return x_out, q_out

def comp_imu_errors(u_in, x_h, simdata):
    if simdata['scalefactors'] == 'on' and simdata['biases'] == 'on':
        # temp = 1 / (1 - x_h[15:])
        temp = 1 / (np.ones(6) - x_h[15:])
        u_out = np.diag(temp) @ u_in + x_h[9:15]
    elif simdata['scalefactors'] == 'on' and simdata['biases'] == 'off':
        temp = 1 / (np.ones(6) - x_h[9:])
        u_out = np.diag(temp) @ u_in
    elif simdata['scalefactors'] == 'off' and simdata['biases'] == 'on':
        u_out = u_in + x_h[9:]
    else:
        u_out = u_in
    return u_out

def q2dcm(q):
    p = np.zeros(6)
    p[0:4] = q ** 2
    p[4] = p[1] + p[2]
    p[5] = 2 / (p[0] + p[3] + p[4]) if (p[0] + p[3] + p[4]) != 0 else 0

    R = np.zeros((3, 3))
    R[0, 0] = 1 - p[5] * p[4]
    R[1, 1] = 1 - p[5] * (p[0] + p[2])
    R[2, 2] = 1 - p[5] * (p[0] + p[1])

    p[0] = p[5] * q[0]
    p[1] = p[5] * q[1]
    p[4] = p[5] * q[2] * q[3]
    p[5] = p[0] * q[1]

    R[0, 1] = p[5] - p[4]
    R[1, 0] = p[5] + p[4]
    p[4] = p[1] * q[3]
    p[5] = p[0] * q[2]
    R[0, 2] = p[5] + p[4]
    R[2, 0] = p[5] - p[4]
    p[4] = p[0] * q[3]
    p[5] = p[1] * q[2]
    R[1, 2] = p[5] - p[4]
    R[2, 1] = p[5] + p[4]

    return R

def dcm2q(R):
    T = 1 + R[0, 0] + R[1, 1] + R[2, 2]

    if T > 1e-8:
        S = 0.5 / np.sqrt(T)
        qw = 0.25 / S
        qx = (R[2, 1] - R[1, 2]) * S
        qy = (R[0, 2] - R[2, 0]) * S
        qz = (R[1, 0] - R[0, 1]) * S
    else:
        if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S

    return np.array([qx, qy, qz, qw])

def Rt2b(ang):
    cr = np.cos(ang[0])
    sr = np.sin(ang[0])
    cp = np.cos(ang[1])
    sp = np.sin(ang[1])
    cy = np.cos(ang[2])
    sy = np.sin(ang[2])

    R = np.array([
        [cy * cp, sy * cp, -sp],
        [-sy * cr + cy * sp * sr, cy * cr + sy * sp * sr, cp * sr],
        [sy * sr + cy * sp * cr, -cy * sr + sy * sp * cr, cp * cr]
    ])
    return R
import numpy as np
import matplotlib.pyplot as plt

from geometrys import q2dcm, dcm2q, Rt2b

class INS:
    def __init__(self, simdata):
        self.simdata = simdata
        self.initial_state = None
        self.init_xh = None
        self.init_quat = None
        self.init_cov = None
        self.init_P = None
        Q, R, H = self.init_QRH()
        self.Q = Q
        self.R = R
        self.H = H

    def init_QRH(self):
        Q = np.zeros((6, 6))
        H = np.zeros((3, 9))
        Q[0:3, 0:3] = np.diag(self.simdata['sigma_acc'].ravel() ** 2)
        Q[3:6, 3:6] = np.diag(self.simdata['sigma_gyro'].ravel() ** 2)
        H[0:3, 3:6] = np.eye(3)
        R = np.diag(self.simdata['sigma_vel'] ** 2)
        return Q, R, H
    
    def init_P_value(self, init_P=None):
        if init_P is not None:
            return init_P
        P = np.zeros((9, 9))
        P[0:3, 0:3] = np.diag(self.simdata['sigma_initial_pos'].ravel() ** 2)
        P[3:6, 3:6] = np.diag(self.simdata['sigma_initial_vel'].ravel() ** 2)
        P[6:9, 6:9] = np.diag(self.simdata['sigma_initial_att'].ravel() ** 2)
        return P
    
    def init_state(self, imu_data, P, init_xh, init_quat):
        N = len(imu_data[0])
        cov = np.zeros((9, N))
        x_h = np.zeros((9, N))
        cov[:, 0] = np.diag(P)
        if init_xh is None and init_quat is None:
            x_h[:, 0], quat = self.init_Nav_eq(imu_data)
        else:
            x_h[:, 0], quat = init_xh, init_quat
        return x_h, quat, cov

    def baseline2(self, imu, zupt, logL, adpt_flag, init_state=None, init_quat=None, init_P=None):
        N = len(imu[0])
        P = self.init_P_value(init_P=init_P)
        x_h, quat, cov = self.init_state(imu, P, init_state, init_quat)
        Id = np.eye(P.shape[0])
        if adpt_flag:
            zupt = np.zeros((1, len(logL[0]) + 5), dtype=bool)
            delta_t = 0
            gamma = np.zeros(len(logL[0]))
            if self.simdata['detector_prio'] == 'normalized velocity':
                c1, c2, c3 = -1e2 * self.simdata['Window_size'] / 2, -5e4 * self.simdata['Window_size'] / 2, 100
            else:
                c1, c2, c3 = -1e2 * self.simdata['Window_size'] / 2, -5e4 * self.simdata['Window_size'] / 2, 0


        for k in range(1, N):
            u_h = self.comp_imu_errors(imu[:, k], x_h[:, k-1])  # Computation of IMU errors
            x_h[:, k], quat = self.Navigation_equations(x_h[:, k-1], u_h, quat)  # Navigation equations
            F, G = self.state_matrix(quat, u_h)  # State matrix calculation

            P = F @ P @ F.T + G @ self.Q @ G.T
            P = (P + P.T) / 2
            cov[:, k] = np.diag(P)

            if adpt_flag:
                S = P[3:6, 3:6]
                v = x_h[3:6, k]

                gamma[k] = c1 + c2 * delta_t + c3 * (v.T @ np.linalg.inv(S) @ v)
                if logL[0][k] > gamma[k]:
                    zupt[0][k:k + self.simdata['Window_size']] = True
                    delta_t = 0
                else:
                    delta_t += self.simdata['Ts']

            if zupt[0][k]:
                K = P @ self.H.T @ np.linalg.inv(self.H @ P @ self.H.T + self.R)
                z = -x_h[3:6, k]
                dx = K @ z
                x_h[:, k], quat = self.comp_internal_states(x_h[:, k], dx, quat)
                P = (Id - K @ self.H) @ P
                P = (P + P.T) / 2
                cov[:, k] = np.diag(P)

        if adpt_flag:
            t = self.simdata['Ts'] * np.arange(N)
            gamma[gamma > -1e2] = -1e1
            plt.figure()
            plt.clf()
            plt.semilogy(t, logL.ravel(), 'k')
            plt.semilogy(t, gamma, 'r')
            plt.yscale('symlog')
            plt.grid(which='minor')
            plt.ylim([-1e8, -1e2])
            plt.show()

        return x_h, cov, quat, P


    def detector_adaptive(self, u):
        """
        Wrapper function for running the zero-velocity detection algorithms.
        """
        zupt = np.zeros((1, (len(u[0]))))

        T = self.GLRT(u)

        # Check if the test statistics T are below the detector threshold
        W = self.simdata['Window_size']
        for k in range(len(T)):
            if T[k] < self.simdata['gamma']:
                zupt[0][k:k+W] = 1

        # Fix the edges of the detector statistics
        T = np.concatenate((np.full(int(np.floor(W/2)), max(T)), T, np.full(int(np.floor(W/2)), max(T))))
        
        # Log-likelihood
        logL = -W / 2 * T

        return zupt, logL.reshape(1,-1)
    
    def GLRT(self, u):
        """
        Function that runs the generalized likelihood test (SHOE detector).
        """
        g = self.simdata['g']
        sigma2_a = self.simdata['sigma_a'] ** 2
        sigma2_g = self.simdata['sigma_g'] ** 2
        W = self.simdata['Window_size']

        N = len(u[0])
        # print(N, '-',W)
        # temp_size = min(N - W + 1,1)
        T = np.zeros(N - W + 1)
        for k in range(N - W + 1):
            ya_m = np.mean(u[0:3, k:k+W], axis=1)
            for l in range(k, k + W):
                tmp = u[0:3, l] - g * ya_m / np.linalg.norm(ya_m)
                T[k] += np.dot(u[3:6, l], u[3:6, l]) / sigma2_g + np.dot(tmp, tmp) / sigma2_a

        T = T / W

        return T


    def baseline(self, u, zupt, logL, adpt_flag, init_state=None, init_quat=None, init_P=None):
        N = len(u[0])
        P, Q, R, H = self.init_filter(init_P=init_P)  # Initialization of filter

        x_h, cov, Id = self.init_vec(N, P)  # Initialization of vectors and matrices
        if init_state is not None:
            x_h[0:9, 0] = init_state
            quat = init_quat
        else:
            x_h[0:9, 0], quat = self.init_Nav_eq(u)  # Initialization of navigation equations

        if adpt_flag:
            zupt = np.zeros((1, len(logL[0]) + 5), dtype=bool)
            delta_t = 0
            gamma = np.zeros(len(logL[0]))
            if self.simdata['detector_prio'] == 'normalized velocity':
                c1, c2, c3 = -1e2 * self.simdata['Window_size'] / 2, -5e4 * self.simdata['Window_size'] / 2, 100
            else:
                c1, c2, c3 = -1e2 * self.simdata['Window_size'] / 2, -5e4 * self.simdata['Window_size'] / 2, 0

        for k in range(1, N):
            u_h = self.comp_imu_errors(u[:, k], x_h[:, k-1])  # Computation of IMU errors
            x_h[:, k], quat = self.Navigation_equations(x_h[:, k-1], u_h, quat)  # Navigation equations
            F, G = self.state_matrix(quat, u_h)  # State matrix calculation

            P = F @ P @ F.T + G @ Q @ G.T
            P = (P + P.T) / 2
            cov[:, k] = np.diag(P)

            if adpt_flag:
                S = P[3:6, 3:6]
                v = x_h[3:6, k]

                gamma[k] = c1 + c2 * delta_t + c3 * (v.T @ np.linalg.inv(S) @ v)
                if logL[0][k] > gamma[k]:
                    zupt[0][k:k + self.simdata['Window_size']] = True
                    delta_t = 0
                else:
                    delta_t += self.simdata['Ts']

            if zupt[0][k]:
                K = P @ H.T @ np.linalg.inv(H @ P @ H.T + R)
                z = -x_h[3:6, k]
                dx = K @ z
                x_h[:, k], quat = self.comp_internal_states(x_h[:, k], dx, quat)
                P = (Id - K @ H) @ P
                P = (P + P.T) / 2
                cov[:, k] = np.diag(P)

        if adpt_flag:
            t = self.simdata['Ts'] * np.arange(N)
            gamma[gamma > -1e2] = -1e1
            plt.figure()
            plt.clf()
            plt.semilogy(t, logL.ravel(), 'k')
            plt.semilogy(t, gamma, 'r')
            plt.yscale('symlog')
            plt.grid(which='minor')
            plt.ylim([-1e8, -1e2])
            plt.show()
            
        return x_h, cov, quat, P

    def init_vec(self, N, P): # checked
        cov = np.zeros((9, N))
        x_h = np.zeros((9, N))
        Id = np.eye(P.shape[0])
        cov[:, 0] = np.diag(P)
        return x_h, cov, Id
    
    def init_Nav_eq(self, u): # checked
        f_u = np.mean(u[0, :20])
        f_v = np.mean(u[1, :20])
        f_w = np.mean(u[2, :20])

        roll = np.arctan2(-f_v, -f_w)
        pitch = np.arctan2(f_u, np.sqrt(f_v ** 2 + f_w ** 2))

        attitude = np.array([roll, pitch, self.simdata['init_heading']])
        Rb2t = Rt2b(attitude).T
        quat = dcm2q(Rb2t)

        x = np.zeros(9)
        x[0:3] = self.simdata['init_pos'].flatten()
        x[6:9] = attitude
        return x, quat

    def init_filter(self, init_P=None): # checked
        P = np.zeros((9, 9))
        Q = np.zeros((6, 6))
        H = np.zeros((3, 9))
        
        if init_P is not None:
            P = init_P
        else:
            P[0:3, 0:3] = np.diag(self.simdata['sigma_initial_pos'].ravel() ** 2)
            P[3:6, 3:6] = np.diag(self.simdata['sigma_initial_vel'].ravel() ** 2)
            P[6:9, 6:9] = np.diag(self.simdata['sigma_initial_att'].ravel() ** 2)


        H[0:3, 3:6] = np.eye(3)
        Q[0:3, 0:3] = np.diag(self.simdata['sigma_acc'].ravel() ** 2)
        Q[3:6, 3:6] = np.diag(self.simdata['sigma_gyro'].ravel() ** 2)
        R = np.diag(self.simdata['sigma_vel'] ** 2)

        return P, Q, R, H
    

    def Navigation_equations(self, x, u, q): # checked
        y = np.zeros_like(x)
        Ts = self.simdata['Ts']

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

        g_t = np.array([0, 0, self.simdata['g']])
        f_t = Rb2t @ u[0:3]
        acc_t = f_t + g_t

        A = np.eye(6)
        A[0, 3] = Ts
        A[1, 4] = Ts
        A[2, 5] = Ts
        B = np.vstack(((Ts ** 2) / 2 * np.eye(3), Ts * np.eye(3)))

        y[0:6] = A @ x[0:6] + B @ acc_t

        return y, q
    
    def state_matrix(self, q, u): # checked
        Rb2t = q2dcm(q)
        f_t = Rb2t @ u[0:3]
        St = np.array([[0, -f_t[2], f_t[1]], [f_t[2], 0, -f_t[0]], [-f_t[1], f_t[0], 0]])

        O = np.zeros((3, 3))
        I = np.eye(3)
        Da = np.diag(u[0:3])
        Dg = np.diag(u[3:6])

        B1 = -1 / self.simdata['acc_bias_instability_time_constant_filter'] * np.eye(3)
        B2 = -1 / self.simdata['gyro_bias_instability_time_constant_filter'] * np.eye(3)

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

        F = np.eye(Fc.shape[0]) + (self.simdata['Ts'] * Fc)
        G = self.simdata['Ts'] * Gc
        return F, G
    
    def comp_internal_states(self, x_in, dx, q_in):
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
    
    def comp_imu_errors(self, u_in, x_h):
        u_out = u_in
        return u_out
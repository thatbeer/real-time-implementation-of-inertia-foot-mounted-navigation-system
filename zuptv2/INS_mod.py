import numpy as np
from omegaconf import OmegaConf

# from detector import detector_adaptive
from geometrys import q2dcm, dcm2q, Rt2b

class INS:
    def __init__(self, simdata, adpt_flag = True) -> None:
        self.simdata = simdata
        self.adpt_flag = adpt_flag
        # self.g = self.gravity(simdata['latitude'],simdata[''])

    def baseline(self, imu_data, zupt, logL, initial_state = None, init_quat=None, initial_P= None):
        N = len(imu_data[0])
        P, Q, R, H = self.init_filter()
        x_h, cov, Id = self.init_vec(N, P)
        # if initial_state is None and init_quat is None:
        #     x_h[0:9, 0], quat = self.init_Nav_eq(imu_data)
        # else:
        #     x_h[0:9, 0] = initial_state
        #     quat = init_quat
        x_h[0:9, 0], quat = self.init_Nav_eq(imu_data)

        if self.adpt_flag:
            zupt = np.zeros((1, len(logL[0] + 5)), dtype=bool)
            delta_t = 0
            gamma = np.zeros(len(logL[0]))
            if self.simdata['detector_prio'] == 'normalized velocity':
                c1, c2, c3 = -1e2 * self.simdata['Window_size'] / 2, -5e4 * self.simdata['Window_size'] / 2, 100
            else:
                c1, c2, c3 = -1e2 * self.simdata['Window_size'] / 2, -5e4 * self.simdata['Window_size'] / 2, 0
        
        for k in range(1, N):
            u_h = self.comp_imu_errors(imu_data[:, k], x_h[:, k-1])
            x_h[:, k] , quat = self.Navigation_equations(x_h[:, k-1], u_h, quat)
            F , G = self.state_matrix(quat, u_h)

            P = F @ P @ F.T + G @ Q @ G.T
            P = (P + P.T) / 2
            cov[:, k] = np.diag(P)            
            
            if self.adpt_flag:
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
                cov[:, k] = np.diag(P) # take last P to use in the next batch (covarnace of the estimate)
        
        if self.adpt_flag:
            t = self.simdata['Ts'] * np.arange(N)
            gamma[gamma > -1e2] = -1e1
            self.t = t
            self.gamma_list = gamma
        
        return x_h , cov, quat, P # change from cov to P
    
    def init_vec(self, N, P):
        if self.simdata['scalefactors'] == 'on' and self.simdata['biases'] == 'on':
            cov = np.zeros((9 + 6 + 6, N))
            x_h = np.zeros((9 + 6 + 6, N))
        elif self.simdata['scalefactors'] == 'on' and self.simdata['biases'] == 'off':
            cov = np.zeros((9 + 6, N))
            x_h = np.zeros((9 + 6, N))
        elif self.simdata['scalefactors'] == 'off' and self.simdata['biases'] == 'on':
            cov = np.zeros((9 + 6, N))
            x_h = np.zeros((9 + 6, N))
        else:
            cov = np.zeros((9, N))
            x_h = np.zeros((9, N))
        Id = np.eye(P.shape[0])
        cov[:, 0] = np.diag(P)
        return x_h, cov, Id
    
    def init_Nav_eq(self, imu_data): # checked
        f_u = np.mean(imu_data[0, :20])
        f_v = np.mean(imu_data[1, :20])
        f_w = np.mean(imu_data[2, :20])

        roll = np.arctan2(-f_v, -f_w)
        pitch = np.arctan2(f_u, np.sqrt(f_v ** 2 + f_w ** 2))


        attitude = np.array([roll, pitch, self.simdata['init_heading']])
        Rb2t = Rt2b(attitude).T
        quat = dcm2q(Rb2t)

        x = np.zeros(9)
        x[0:3] = self.simdata['init_pos'].flatten()
        x[6:9] = attitude
        return x, quat

    def init_filter(self, initial_cov=None): # checked
        if self.simdata['scalefactors'] == 'on' and self.simdata['biases'] == 'on':
            P = np.zeros((9 + 6 + 6, 9 + 6 + 6))
            P[9:12, 9:12] = np.diag(self.simdata['sigma_initial_acc_bias'] ** 2)
            P[12:15, 12:15] = np.diag(self.simdata['sigma_initial_gyro_bias'] ** 2)
            P[15:18, 15:18] = np.diag(self.simdata['sigma_initial_acc_scale'] ** 2)
            P[18:21, 18:21] = np.diag(self.simdata['sigma_initial_gyro_scale'] ** 2)
            Q = np.zeros((12, 12))
            Q[6:9, 6:9] = np.diag(self.simdata['acc_bias_driving_noise'] ** 2)
            Q[9:12, 9:12] = np.diag(self.simdata['gyro_bias_driving_noise'] ** 2)
            H = np.zeros((3, 9 + 6 + 6))
        elif self.simdata['scalefactors'] == 'on' and self.simdata['biases'] == 'off':
            P = np.zeros((9 + 6, 9 + 6))
            P[9:12, 9:12] = np.diag(self.simdata['sigma_initial_acc_scale'] ** 2)
            P[12:15, 12:15] = np.diag(self.simdata['sigma_initial_gyro_scale'] ** 2)
            Q = np.zeros((6, 6))
            H = np.zeros((3, 9 + 6))
        elif self.simdata['scalefactors'] == 'off' and self.simdata['biases'] == 'on':
            P = np.zeros((9 + 6, 9 + 6))
            P[9:12, 9:12] = np.diag(self.simdata['sigma_initial_acc_bias'] ** 2)
            P[12:15, 12:15] = np.diag(self.simdata['sigma_initial_gyro_bias'] ** 2)
            Q = np.zeros((12, 12))
            Q[6:9, 6:9] = np.diag(self.simdata['acc_bias_driving_noise'] ** 2)
            Q[9:12, 9:12] = np.diag(self.simdata['gyro_bias_driving_noise'] ** 2)
            H = np.zeros((3, 9 + 6))
        else:
            if initial_cov is None:
                P = np.zeros((9, 9))
            else:
                P = initial_cov # should have size of (9, 9)
            Q = np.zeros((6, 6))
            H = np.zeros((3, 9))

        if initial_cov is None: 
            P[0:3, 0:3] = np.diag(self.simdata['sigma_initial_pos'] ** 2)
            P[3:6, 3:6] = np.diag(self.simdata['sigma_initial_vel'] ** 2)
            P[6:9, 6:9] = np.diag(self.simdata['sigma_initial_att'] ** 2)
        
        Q[0:3, 0:3] = np.diag(self.simdata['sigma_acc'] ** 2)
        Q[3:6, 3:6] = np.diag(self.simdata['sigma_gyro'] ** 2)
        R = np.diag(self.simdata['sigma_vel'] ** 2)
        H[0:3, 3:6] = np.eye(3)

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

        if self.simdata['scalefactors'] == 'on' and self.simdata['biases'] == 'on':
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
        elif self.simdata['scalefactors'] == 'on' and self.simdata['biases'] == 'off':
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
        elif self.simdata['scalefactors'] == 'off' and self.simdata['biases'] == 'on':
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

        F = np.eye(Fc.shape[0]) + (self.simdata['Ts'] * Fc)
        G = self.simdata['Ts'] * Gc
        return F, G


    def comp_internal_states(self, x_in, dx, q_in):
        R = q2dcm(q_in)
        x_out = x_in + dx
        epsilon = dx[6:9]

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
        if self.simdata['scalefactors'] == 'on' and self.simdata['biases'] == 'on':
            # temp = 1 / (1 - x_h[15:])
            temp = 1 / (np.ones(6) - x_h[15:])
            u_out = np.diag(temp) @ u_in + x_h[9:15]
        elif self.simdata['scalefactors'] == 'on' and self.simdata['biases'] == 'off':
            temp = 1 / (np.ones(6) - x_h[9:])
            u_out = np.diag(temp) @ u_in
        elif self.simdata['scalefactors'] == 'off' and self.simdata['biases'] == 'on':
            u_out = u_in + x_h[9:]
        else:
            u_out = u_in
        return u_out
    
    # detector side
    def detector(self, u):
        zupt = np.zeros((1, (len(u[0]))))
        # Run the desired detector type
        T = self.GLRT(u)
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
        T = np.zeros(N - W + 1)
        for k in range(N - W + 1):
            ya_m = np.mean(u[0:3, k:k+W], axis=1)
            for l in range(k, k + W):
                tmp = u[0:3, l] - g * ya_m / np.linalg.norm(ya_m)
                T[k] += np.dot(u[3:6, l], u[3:6, l]) / sigma2_g + np.dot(tmp, tmp) / sigma2_a

        T = T / W

        return T
    
    @staticmethod
    def gravity(latitude, altitude):
        lambda_rad = np.pi / 180 * latitude
        gamma = 9.780327 * (1 + 0.0053024 * np.sin(lambda_rad) ** 2 - 0.0000058 * np.sin(2 * lambda_rad) ** 2)
        g = gamma - ((3.0877e-6) - (0.004e-6) * np.sin(lambda_rad) ** 2) * altitude + (0.072e-12) * altitude ** 2
        return g
import numpy as np

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
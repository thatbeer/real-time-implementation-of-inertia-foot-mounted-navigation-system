import numpy as np

def detector_adaptive(u, simdata):
    """
    Wrapper function for running the zero-velocity detection algorithms.
    """
    zupt = np.zeros((1, (len(u[0]))))

    # Run the desired detector type
    if simdata['detector_type'] == 'GLRT':
        T = GLRT(u, simdata)
    elif simdata['detector_type'] == 'MV':
        T = MV(u, simdata)
    elif simdata['detector_type'] == 'MAG':
        T = MAG(u, simdata)
    elif simdata['detector_type'] == 'ARE':
        T = ARE(u, simdata)
    else:
        print('The chosen detector type is not recognized. The GLRT detector is used')
        T = GLRT(u, simdata)

    # Check if the test statistics T are below the detector threshold
    W = simdata['Window_size']
    for k in range(len(T)):
        if T[k] < simdata['gamma']:
            zupt[0][k:k+W] = 1

    # Fix the edges of the detector statistics
    T = np.concatenate((np.full(int(np.floor(W/2)), max(T)), T, np.full(int(np.floor(W/2)), max(T))))
    
    # Log-likelihood
    logL = -W / 2 * T

    return zupt, logL.reshape(1,-1)

def GLRT(u, simdata):
    """
    Function that runs the generalized likelihood test (SHOE detector).
    """
    g = simdata['g']
    sigma2_a = simdata['sigma_a'] ** 2
    sigma2_g = simdata['sigma_g'] ** 2
    W = simdata['Window_size']

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

def MV(u, simdata):
    """
    Function that runs the acceleration moving variance detector.
    """
    sigma2_a = simdata['sigma_a'] ** 2
    W = simdata['Window_size']

    N = len(u[0])
    T = np.zeros(N - W + 1)

    for k in range(N - W + 1):
        ya_m = np.mean(u[0:3, k:k+W], axis=1)
        for l in range(k, k + W):
            tmp = u[0:3, l] - ya_m
            T[k] += np.dot(tmp, tmp)

    T = T / (sigma2_a * W)

    return T

def MAG(u, simdata):
    """
    Function that runs the acceleration magnitude detector.
    """
    g = simdata['g']
    sigma2_a = simdata['sigma_a'] ** 2
    W = simdata['Window_size']

    N = len(u[0])
    T = np.zeros(N - W + 1)

    for k in range(N - W + 1):
        for l in range(k, k + W):
            T[k] += (np.linalg.norm(u[0:3, l]) - g) ** 2

    T = T / (sigma2_a * W)

    return T

def ARE(u, simdata):
    """
    Function that runs the angular rate energy detector.
    """
    sigma2_g = simdata['sigma_g'] ** 2
    W = simdata['Window_size']

    N = len(u[0])
    T = np.zeros(N - W + 1)

    for k in range(N - W + 1):
        for l in range(k, k + W):
            T[k] += np.linalg.norm(u[3:6, l]) ** 2

    T = T / (sigma2_g * W)

    return T

# Example usage
# simdata = {
#     'detector_type': 'GLRT',
#     'Window_size': 5,
#     'g': 9.82,
#     'sigma_a': 0.01,
#     'sigma_g': 0.1 * np.pi / 180,
#     'gamma': 0.3e5
# }
# u = np.random.randn(6, 1000)  # Example IMU data
# zupt, logL = detector_adaptive(u, simdata)
# print(zupt)
# print(logL)

import numpy as np

def detector_adaptive(u, simdata):
    """
    Wrapper function for running the zero-velocity detection algorithms.
    """
    # Allocate memory
    zupt = np.zeros(len(u))

    # Run the desired detector type. Each detector returns a vector with their 
    # calculated test statistics T.
    if simdata['detector_type'] == 'GLRT':
        T = GLRT(u, simdata)
    elif simdata['detector_type'] == 'MV':
        T = MV(u, simdata)
    elif simdata['detector_type'] == 'MAG':
        T = MAG(u, simdata)
    elif simdata['detector_type'] == 'ARE':
        T = ARE(u, simdata)

    # Calculate the test statistics
    logL = np.log(T)

    # Make decision based on threshold
    threshold = simdata['zupt_threshold']
    zupt = logL < threshold

    return zupt, logL

def GLRT(u, simdata):
    """
    Function that runs the Generalized Likelihood Ratio Test (GLRT).
    """
    sigma2_a = simdata['sigma_a'] ** 2
    W = simdata['Window_size']

    N = len(u)
    print(N)
    T = np.zeros(N - W + 1)

    for k in range(N - W + 1):
        ya_m = np.mean(u[:3, k:k + W], axis=1)
        for l in range(k, k + W):
            tmp = u[:3, l] - ya_m
            T[k] += np.dot(tmp, tmp)
    
    T /= (sigma2_a * W)
    return T

def MV(u, simdata):
    """
    Function that runs the acceleration moving variance detector.
    """
    sigma2_a = simdata['sigma_a'] ** 2
    W = simdata['Window_size']

    N = len(u)
    T = np.zeros(N - W + 1)

    for k in range(N - W + 1):
        ya_m = np.mean(u[:3, k:k + W], axis=1)
        for l in range(k, k + W):
            tmp = u[:3, l] - ya_m
            T[k] += np.dot(tmp, tmp)
    
    T /= (sigma2_a * W)
    return T

def MAG(u, simdata):
    """
    Function that runs the acceleration magnitude detector.
    """
    g = simdata['g']
    sigma2_a = simdata['sigma_a'] ** 2
    W = simdata['Window_size']

    N = len(u)
    T = np.zeros(N - W + 1)

    for k in range(N - W + 1):
        for l in range(k, k + W):
            T[k] += (np.linalg.norm(u[:3, l]) - g) ** 2
    
    T /= (sigma2_a * W)
    return T

def ARE(u, simdata):
    """
    Function that runs the angular rate energy detector.
    """
    sigma2_g = simdata['sigma_g'] ** 2
    W = simdata['Window_size']

    N = len(u)
    T = np.zeros(N - W + 1)

    for k in range(N - W + 1):
        for l in range(k, k + W):
            T[k] += np.linalg.norm(u[3:6, l]) ** 2
    
    T /= (sigma2_g * W)
    return T

import numpy as np

def gravity(latitude, altitude):
    """
    Function for calculating the magnitude of the local gravity.
    
    Args:
    latitude (float): Latitude in degrees.
    altitude (float): Altitude in meters.
    
    Returns:
    float: Magnitude of the local gravity vector in m/s^2.
    """
    lambda_rad = np.pi / 180 * latitude
    gamma = 9.780327 * (1 + 0.0053024 * np.sin(lambda_rad) ** 2 - 0.0000058 * np.sin(2 * lambda_rad) ** 2)
    g = gamma - ((3.0877e-6) - (0.004e-6) * np.sin(lambda_rad) ** 2) * altitude + (0.072e-12) * altitude ** 2
    return g

def Init_det_glrt():
    """
    Initialize the simulation data parameters.
    
    Returns:
        dict: A dictionary containing all simulation parameters.
    """
    simdata = {}

    #############################################
    ##              GENERAL PARAMETERS         ## 
    #############################################

    # Rough altitude [m]
    simdata['altitude'] = 100

    # Rough latitude [degrees]
    simdata['latitude'] = 58

    # Magnitude of the local gravity vector [m/s^2]
    simdata['g'] = gravity(simdata['latitude'], simdata['altitude'])

    # Sampling period [s]
    simdata['Ts'] = 1 / 250 # alogrithm configuration
    # Sampling rate [Hz]
    simdata['Hz'] = 250 # sensor configuration

    # Initial heading [rad]
    simdata['init_heading'] = 0 * np.pi / 180

    # Initial position (x,y,z)-axis [m]
    simdata['init_pos'] = np.array([0, 0, 0]).reshape(-1, 1)

    # Data sets
    simdata['data_sets'] = np.arange(42, 62, 1)  # Fast

    #############################################
    ##           Detector Settings             ## 
    #############################################

    # Detector type
    simdata['detector_type'] = 'GLRT'

    # Standard deviation of the accelerometer noise [m/s^2]
    simdata['sigma_a'] = 0.01

    # Standard deviation of the gyroscope noise [rad/s]
    simdata['sigma_g'] = 0.1 * np.pi / 180

    # Window size of the zero-velocity detector [samples]
    simdata['Window_size'] = 5 # originally at 5

    # Threshold used in the zero-velocity detector
    simdata['gamma'] = 0.3e5

    # Prio to use in the adaptive threshold
    simdata['detector_prio'] = 'normalized velocity'

    #############################################
    ##             FILTER PARAMETERS           ## 
    #############################################

    # Include scale factors
    simdata['biases'] = 'off'
    simdata['scalefactors'] = 'off'

    # Process noise covariance (Q)
    simdata['sigma_acc'] = 1.3 * np.array([1, 1, 1]).reshape(-1, 1)  # [m/s^2]
    simdata['sigma_gyro'] = 0.01 * np.array([1, 1, 1]).reshape(-1, 1) * np.pi / 180  # [rad/s]
    simdata['acc_bias_driving_noise'] = 0.0000001 * np.ones((3,1))  # [m/s^2]
    simdata['gyro_bias_driving_noise'] = 0.0000001 * np.pi / 180  # [rad/s]

    # Pseudo zero-velocity update measurement noise covariance (R)
    simdata['sigma_vel'] = np.array([0.1, 0.1, 0.1])  # [m/s]

    # Diagonal elements of the initial state covariance matrix (P)
    simdata['sigma_initial_pos'] = 1e-5 * np.ones(3)  # Position (x,y,z navigation coordinate axis) [m]
    simdata['sigma_initial_vel'] = 1e-5 * np.ones(3)  # Velocity (x,y,z navigation coordinate axis) [m/s]
    simdata['sigma_initial_att'] = (np.pi / 180 * np.array([1, 1, 0.1])).reshape(-1, 1)  # Attitude (roll, pitch, heading) [rad]
    simdata['sigma_initial_acc_bias'] = 0.3 * np.ones(3)  # Accelerometer biases (x,y,z platform coordinate axis) [m/s^2]
    simdata['sigma_initial_gyro_bias'] = 0.3 * np.pi / 180 * np.ones(3)  # Gyroscope biases (x,y,z platform coordinate axis) [rad/s]
    simdata['sigma_initial_acc_scale'] = 0.0001 * np.ones(3)  # Accelerometer scale factors (x,y,z platform coordinate axis)
    simdata['sigma_initial_gyro_scale'] = 0.00001 * np.ones(3)  # Gyroscope scale factors (x,y,z platform coordinate axis)

    # Bias instability time constants [seconds]
    simdata['acc_bias_instability_time_constant_filter'] = float('inf')
    simdata['gyro_bias_instability_time_constant_filter'] = float('inf')

    return simdata

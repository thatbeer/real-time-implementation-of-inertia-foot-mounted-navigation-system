import os
import pandas as pd
import numpy as np

def func_loaddataset(str_path):
    """
    Load dataset from a given file path.

    Parameters:
    str_path (str): File path for the dataset.

    Returns:
    u (numpy.ndarray): Array containing IMU data (accelerometer and gyroscope).
    """

    # Define file path for inertial data
    # str_imu = 'data_inert.txt'
    # front = 'ZUPTaidedINS'
    # temp = os.path.join(front, str_path)
    # full_path = os.path.join(str_path, str_imu)

    # # Load inertial data
    # with open(full_path, 'r') as data_inert_file:
    #     # Skip headers (assuming 32 headers as in the original MATLAB code)
    #     for _ in range(32):
    #         next(data_inert_file)
        
    #     # Load data set
    #     data_inert = np.genfromtxt(data_inert_file, delimiter='  ', dtype=None, encoding=None)

    column_names = ['Header', 'a_x [g]', 'a_y [g]', 'a_z [g]', 
                'ar_x [rad/s]', 'ar_y [rad/s]', 'ar_z [rad/s]', 
                'Counter', 'Checksum', 'Nr', 'Filtered ts[s]', 
                'Unfiltered ts[s]', 'Pred. ts[s]', 'Freq. [Hz]', 
                'm_x [G]', 'm_y [G]', 'm_z [G]']
    data = pd.read_csv(str_path, delimiter='\s+', skiprows=4, names=column_names)
    data = data.to_numpy()

    # Store IMU data in vectors
    imu_scalefactor = 9.80665  # Verify in Microstrain IMU data sheet

    # Extracting and scaling IMU accelerometer data
    f_imu = data[:, 1:4].T * imu_scalefactor

    # Extracting IMU gyroscope data
    omega_imu = data[:, 4:7].T

    # Combine accelerometer and gyroscope data
    u = np.vstack([f_imu, omega_imu])

    return u
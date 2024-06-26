import matplotlib.pyplot as plt
import numpy as np

def plot_position_tracking(imu_data):
    """
    Plots the 2D top view of position tracking from IMU data.

    Parameters:
    imu_data (numpy.ndarray): IMU data array of shape (9, N) where the first two rows are x and y coordinates.
    """
    # Extract x and y positions
    x_positions = imu_data[0, :]  # X coordinates
    y_positions = imu_data[1, :]  # Y coordinates

    # Create the plot
    plt.figure(figsize=(10, 8))
    plt.plot(x_positions, y_positions, marker='o', linestyle='-', color='b', zorder=1)
    plt.title('2D Top View Position Tracking')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.grid(True)
    plt.axis('equal')  # This ensures that unit increments are equal on both axes

    # Highlight the first position with higher zorder
    plt.scatter(x_positions[0], y_positions[0], color='green', s=100, edgecolor='black', label='Start', zorder=2)
    # Highlight the last position with higher zorder
    plt.scatter(x_positions[-1], y_positions[-1], color='red', s=100, edgecolor='black', label='End', zorder=2)

    # Add a legend to identify the start and end
    plt.legend()

    # Display the plot
    plt.show()

def plot_z_position_tracking(imu_data):
    """
    Plots the changes in the z-position over time from IMU data.

    Parameters:
    imu_data (numpy.ndarray): IMU data array of shape (9, N) where the third row is the z coordinate.
    """
    # Extract z positions
    z_positions = imu_data[2, :]  # Z coordinates

    # Create the plot
    plt.figure(figsize=(10, 8))
    plt.plot(z_positions, marker='o', linestyle='-', color='c')  # 'c' for cyan color
    plt.title('Z-Position Tracking Over Time')
    plt.xlabel('Time or Sequence Index')
    plt.ylabel('Z Position')
    plt.grid(True)

    # Display the plot
    plt.show()

def load_imu_data(file_path):
    """
    Load and convert IMU data from a given CSV file.

    Parameters:
    file_path (str): Path to the CSV file containing IMU data.

    Returns:
    numpy.ndarray: Transposed array containing converted IMU data.
    """
    # Read the dataset assuming comma-separated values without headers
    data = pd.read_csv(file_path, header=None, names=['acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z'])
    # Create an array of the modified data
    u = data.to_numpy().T

    return u

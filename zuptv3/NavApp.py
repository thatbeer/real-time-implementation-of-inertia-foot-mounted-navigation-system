# This version use INS without batch processing window, 
# this could lead to slow performacne due to the amount of
# data accmulated over time.
import sys
import os
import threading
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk
import xsensdeviceapi as xda

from omegaconf import OmegaConf

from Init_det_glrt import Init_det_glrt
# from INS import INS
# from RTINS import INS
from INSV2 import INS
# from ZUPTaidedINS import ZUPTaidedINS
# from detector import detector_adaptive
# from xda_utils import *
# from INS import INS
from xda_class import XdaCallback, Scanner



def generate_unique_filename(base_name, extension=".csv"):
    counter = 1
    filename = f"{base_name}{extension}"
    while os.path.exists(filename):
        filename = f"{base_name}_{counter}{extension}"
        counter += 1
    return filename

def load_config(file_path):
    return OmegaConf.load(file_path)

config = load_config('config.yaml')


class NavigationApp(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("Real-Time Foot-Mounted Navigation")
        self.geometry("1200x800")

        self.config = config
        # self.sampling_rate = config.general_parameters.sample_rate
        self.scanner = Scanner()
        self.callback = XdaCallback()
        self.simdata = Init_det_glrt()
        self.ins = INS(self.simdata)
        # self.simdata = Init_det_glrt()
        self.running = False
        self.device = None
        self.n = 0

        self.sampling_rate = self.simdata['Hz']
        # self.time_window = 1.0 # sec
        self.time_window = self.sampling_rate # sec
        # self.sampling_rate = 1 / self.simdata['Ts']
        self.Ts = self.simdata['Ts']
        self.u_window = []
        self.batch_position = np.zeros((3, 1))
        self.state_vector = np.zeros((9, 1))
        # self.cov = None
        self.state_position = []

        self.init_P = None
        self.init_quat = None
        self.init_position = None
        self.init_dT = 0

        self.positions = []

        self.imu_datas = []

        self.init_ui()

    def init_ui(self):
        # Connection control frame
        self.conn_frame = ttk.LabelFrame(self, text="Connection Control")
        self.conn_frame.grid(row=0, column=0, columnspan=3, padx=10, pady=5, sticky="ew")

        self.connect_button = ttk.Button(self.conn_frame, text="Connect", command=self.connect_sensor)
        self.connect_button.pack(side=tk.LEFT, padx=10, pady=10)

        self.disconnect_button = ttk.Button(self.conn_frame, text="Disconnect", command=self.disconnect_sensor, state=tk.DISABLED)
        self.disconnect_button.pack(side=tk.LEFT, padx=10, pady=10)

        # Sensor information label
        self.info_label = ttk.Label(self.conn_frame, text="Sensor Information: Not Connected")
        self.info_label.pack(side=tk.LEFT, padx=10, pady=10)

        # Data collection control frame
        self.ctrl_frame = ttk.LabelFrame(self, text="Data Collection Control")
        self.ctrl_frame.grid(row=1, column=0, columnspan=3, padx=10, pady=5, sticky="ew")

        self.start_button = ttk.Button(self.ctrl_frame, text="Start", command=self.start_collection, state=tk.DISABLED)
        self.start_button.pack(side=tk.LEFT, padx=10, pady=10)

        self.stop_button = ttk.Button(self.ctrl_frame, text="Stop", command=self.stop_collection, state=tk.DISABLED)
        self.stop_button.pack(side=tk.LEFT, padx=10, pady=10)

        # Add Reset Orientation button
        self.reset_orientation_button = ttk.Button(self.ctrl_frame, text="Reset Orientation", command=self.reset_orientation, state=tk.DISABLED)
        self.reset_orientation_button.pack(side=tk.LEFT, padx=10, pady=10)

        # Position figure
        self.fig_position, self.ax_position = plt.subplots()
        self.ax_position.set_xlim([-10, 10])
        self.ax_position.set_ylim([-10, 10])
        self.ax_position.axhline(0, color='black', linewidth=0.5)
        self.ax_position.axvline(0, color='black', linewidth=0.5)
        self.ax_position.set_xlabel('X Position (m)')
        self.ax_position.set_ylabel('Y Position (m)')
        self.ax_position.set_title('Real-Time Foot-Mounted Navigation')

        self.canvas_position = FigureCanvasTkAgg(self.fig_position, master=self)
        self.canvas_position.get_tk_widget().grid(row=2, column=0, rowspan=3, padx=10, pady=10, sticky="nsew")

        # Orientation figure
        self.fig_orientation = plt.figure()
        self.ax_orientation = self.fig_orientation.add_subplot(111, projection='3d')
        self.ax_orientation.set_xlim([-1, 1])
        self.ax_orientation.set_ylim([-1, 1])
        self.ax_orientation.set_zlim([-1, 1])
        self.ax_orientation.set_xlabel('X')
        self.ax_orientation.set_ylabel('Y')
        self.ax_orientation.set_zlabel('Z')
        self.ax_orientation.set_title('Orientation')

        self.canvas_orientation = FigureCanvasTkAgg(self.fig_orientation, master=self)
        self.canvas_orientation.get_tk_widget().grid(row=2, column=1, padx=10, pady=10, sticky="nsew")

        # Data labels
        self.data_label = ttk.Label(self, text="", anchor="center")
        self.data_label.grid(row=3, column=1, padx=10, pady=5, sticky="ew")

        self.data_text = tk.Text(self, height=10)
        self.data_text.grid(row=4, column=1, padx=10, pady=10, sticky="nsew")

        # Configure grid weights
        self.grid_columnconfigure(0, weight=2)
        self.grid_columnconfigure(1, weight=1)
        self.grid_columnconfigure(2, weight=1)
        self.grid_rowconfigure(2, weight=1)
        self.grid_rowconfigure(3, weight=1)
        self.grid_rowconfigure(4, weight=1)

        # Start the orientation update loop
        self.update_orientation()
    
    def reset_orientation(self):
        if self.device:
            print("Resetting device orientation...")
            self.device.gotoConfig()
            print("Reset the orientation") # XRM_Inclination / XRM_Global
            self.device.resetOrientation(xda.XRM_Global)  ##how to extract the enum to request command?
            print("go back to measurement mode...")
            self.device.gotoMeasurement()
        else:
            print("No device connected.")

    def connect_sensor(self):
        self.device = self.scanner.scan_and_open()  # open port
        self.device.addCallbackHandler(self.callback)  # add Callback
        self.configure_device()  # configure setting

        # Display sensor information
        device_info = self.scanner.get_device_info()
        info_str = ', '.join([f"{key}: {value}" for key, value in device_info.items()])
        self.info_label.config(text=f"Sensor Information: {info_str}")

        self.connect_button.config(state=tk.DISABLED)
        self.disconnect_button.config(state=tk.NORMAL)
        self.start_button.config(state=tk.NORMAL)
        self.reset_orientation_button.config(state=tk.NORMAL)

    def disconnect_sensor(self):
        if self.device:
            self.device.stopRecording()
            self.scanner.close()
        self.connect_button.config(state=tk.NORMAL)
        self.disconnect_button.config(state=tk.DISABLED)
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.DISABLED)
        self.reset_orientation_button.config(state=tk.DISABLED)
        self.reset_initial_position()
        self.add_text(f"Disconnected successfully...")
        self.info_label.config(text="Sensor Information: Not Connected")

    def reset_initial_parameters(self):
        self.init_dT = 0
        self.init_quat = None
        self.init_P = None
        self.init_position = None
        return self
        
    def start_collection(self):
        self.running = True
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)

        self.state_position.clear()  # clear the previous collection
        self.reset_initial_parameters()

        self.collection_thread = threading.Thread(target=self.collect_data)
        self.collection_thread.start()

    def stop_collection(self):
        self.running = False
        self.save_data()
        self.imu_datas.clear()
        if self.collection_thread.is_alive():
            self.collection_thread.join()
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)

    def save_data(self):
        base_name = "imu_data.csv"
        filename = generate_unique_filename(base_name)
        with open(filename, 'w') as file:
            # Assuming 'u' contains 6 elements: 3 from accelerometer and 3 from gyroscope
            # file.write("AccX,AccY,AccZ,GyroX,GyroY,GyroZ\n")
            for data in self.imu_datas:
                file.write(','.join(map(str, data)) + '\n')
        print(f"Data successfully saved to {filename}")
        self.add_text(f"Data successfully saved to {filename}")

    def reset_initial_position(self):
        self.batch_position = np.zeros((3, 1))
        self.state_vector = np.zeros((9, 1))

    def add_text(self, s):
        self.data_label.config(text=s)
        self.data_text.insert(tk.END, s + '\n')
        self.data_text.see(tk.END)  # Scroll to the end of the text box
        return self

    def collect_data(self): 
        while self.running:
            # window_size = int(self.time_window * self.sampling_rate) # 
            window_size = 750 # modified manually 
            if self.callback.packetAvailable():
                s = ""
                packet = self.callback.getNextPacket()
                u = self.process_data(packet)
                s += "Acc X: %.2f" % u[0] + ", Acc Y: %.2f" % u[1] + ", Acc Z: %.2f" % u[2]
                s += "\nGyrp X: %.2f" % u[3] + ", Gyrp Y: %.2f" % u[4] + ", Gyrp Z: %.2f" % u[5]
                self.data_label.config(text=s)
                self.data_text.insert(tk.END, s + '\n')
                self.data_text.see(tk.END)  # Scroll to the end of the text box

                self.u_window.append(u)
                self.imu_datas.append(u)
                self.n += 1
                if self.n >= window_size:
                # if len(self.u_window) > window_size:
                    self.n = 0 # set counter back to 0
                    us = np.array(self.u_window) # batch processing
                    # us = np.array(self.imu_datas) # stacked processing
                    zupt, logL = self.ins.detector_adaptive(us.T)
                    x_h, _, quat, P = self.ins.baseline(
                        us.T, zupt, logL, True, 
                        init_state=self.init_position,
                        init_quat=self.init_quat,
                        init_P=self.init_P,
                        init_dT=self.init_dT)
                    for j in range(len(x_h[0])):
                            xj,yj,zj = x_h[:3, j]
                            self.positions.append([xj, yj, zj])
                    # x, y, z = x_h[0:3, -1]
                    # self.batch_position[0] += x
                    # self.batch_position[1] += y
                    # self.batch_position[2] += z
                    # self.state_position.append(self.batch_position.ravel().tolist())
                    self.init_position, self.init_quat, self.init_P = x_h[:,-1], quat, P
                    self.u_window.clear() # does not need with non-batch
                    self.visualize_data()
                print("%s\r" % s, end="", flush=True)
            else:
                time.sleep(0.01)  # Wait briefly if no packet is available

    def process_data(self, data_packet):
        acc = data_packet.calibratedAcceleration() # 21/6/24 clarification from eng : m/s^2
        gyro = data_packet.calibratedGyroscopeData() # clarification from eng : rad/sec
        return np.array([acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]])

    def visualize_data(self):
        if len(self.positions) > 0:
            # pos = np.array(self.state_position)
            pos = np.array(self.positions)
            x_pos = pos[:, 0]
            y_pos = pos[:, 1]
            
            self.ax_position.clear()
            # self.ax_position.plot(pos[:, 0], pos[:, 1])
            self.ax_position.plot(x_pos, y_pos, marker='o', linestyle='-', color='b', zorder=1)

            max_range = np.max(np.abs(pos)) * 1.1 + 3
            self.ax_position.set_xlim([-max_range, max_range])
            self.ax_position.set_ylim([-max_range, max_range])
            self.ax_position.axhline(0, color='black', linewidth=0.5)
            self.ax_position.axvline(0, color='black', linewidth=0.5)
            self.ax_position.set_xlabel('X Position (m)')
            self.ax_position.set_ylabel('Y Position (m)')
            self.ax_position.set_title('Real-Time Foot-Mounted Navigation')
            self.canvas_position.draw()

    def visualize_orientation(self, packet):
        if packet.containsOrientation():
            quaternion = packet.orientationQuaternion()
            q0, q1, q2, q3 = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
            
            # Convert quaternion to rotation matrix
            R_matrix = np.array([
                [1 - 2 * (q2 ** 2 + q3 ** 2), 2 * (q1 * q2 - q3 * q0), 2 * (q1 * q3 + q2 * q0)],
                [2 * (q1 * q2 + q3 * q0), 1 - 2 * (q1 ** 2 + q3 ** 2), 2 * (q2 * q3 - q1 * q0)],
                [2 * (q1 * q3 - q2 * q0), 2 * (q2 * q3 + q1 * q0), 1 - 2 * (q1 ** 2 + q2 ** 2)]
            ])

            self.ax_orientation.clear()

            # Transform the coordinate frame based on the orientation
            self.ax_orientation.quiver(0, 0, 0, R_matrix[0, 0], R_matrix[0, 1], R_matrix[0, 2], color='r')
            self.ax_orientation.quiver(0, 0, 0, R_matrix[1, 0], R_matrix[1, 1], R_matrix[1, 2], color='g')
            self.ax_orientation.quiver(0, 0, 0, R_matrix[2, 0], R_matrix[2, 1], R_matrix[2, 2], color='b')

            self.ax_orientation.set_xlim([-1, 1])
            self.ax_orientation.set_ylim([-1, 1])
            self.ax_orientation.set_zlim([-1, 1])
            self.ax_orientation.set_xlabel('X')
            self.ax_orientation.set_ylabel('Y')
            self.ax_orientation.set_zlabel('Z')
            self.ax_orientation.set_title('Orientation')
            self.canvas_orientation.draw()

    def update_orientation(self):
        if self.callback.packetAvailable():
            s = ""
            packet = self.callback.getNextPacket()
            self.visualize_orientation(packet)
            # if packet.containsCalibratedData():
            if not self.running:
                acc = packet.calibratedAcceleration()
                s = "Acc X: %.2f" % acc[0] + ", Acc Y: %.2f" % acc[1] + ", Acc Z: %.2f" % acc[2]
                gyr = packet.calibratedGyroscopeData()
                s += " |Gyr X: %.2f" % gyr[0] + ", Gyr Y: %.2f" % gyr[1] + ", Gyr Z: %.2f" % gyr[2]
            self.data_label.config(text=s)
            self.data_text.insert(tk.END, s + '\n')
            self.data_text.see(tk.END)  # Scroll to the end of the text box
        self.after(100, self.update_orientation)  # Update every 100 ms

    def configure_device(self):
        print("Putting device into configuration mode...")
        if not self.device.gotoConfig():
            raise RuntimeError("Could not put device into configuration mode. Aborting.")
        configArray = xda.XsOutputConfigurationArray()
        configArray.push_back(xda.XsOutputConfiguration(xda.XDI_PacketCounter, 0))
        configArray.push_back(xda.XsOutputConfiguration(xda.XDI_SampleTimeFine, 0))
        configArray.push_back(xda.XsOutputConfiguration(xda.XDI_Acceleration, self.sampling_rate))  # 100Hz
        configArray.push_back(xda.XsOutputConfiguration(xda.XDI_RateOfTurn, self.sampling_rate))  # 100Hz
        configArray.push_back(xda.XsOutputConfiguration(xda.XDI_Quaternion, self.sampling_rate))  # 100Hz

        if not self.device.setOutputConfiguration(configArray):
            raise RuntimeError("Failed to configure device")

        print("Creating a log file...")
        logFileName = "logfile.mtb"
        if self.device.createLogFile(logFileName) != xda.XRV_OK:
            raise RuntimeError("Failed to create a log file. Aborting.")
        else:
            print("Created a log file: %s" % logFileName)

        print("Putting device into measurement mode...")
        if not self.device.gotoMeasurement():
            raise RuntimeError("Could not put device into measurement mode. Aborting.")

        print("Starting recording...")
        if not self.device.startRecording():
            raise RuntimeError("Failed to start recording. Aborting.")



if __name__ == "__main__":
    app = NavigationApp()
    app.mainloop()

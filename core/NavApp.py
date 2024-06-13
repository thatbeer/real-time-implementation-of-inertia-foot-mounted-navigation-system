import os
import sys
import threading
import time
import numpy as np
import tkinter as tk
import xsensdeviceapi as xda
import matplotlib.pyplot as plt

from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from threading import Lock
from omegaconf import OmegaConf

from INS import INS

from xda_utils import Scanner , XdaCallback


class NavApp(tk.Tk):
    def __init__(self, config_path='config.yaml'):
        super().__init__()

        self.title("Real-Time Foot-Mounted Navigation")
        self.geometry("1200x800")


        self.config = self.load_config(config_path)
        self.sample_rate = self.config.general_parameters.sample_rate
        self.time_window = 1.0
        self.u_window = []
        self.current_position = np.zeros(3) # x,y,z
        self.state_vector = np.zeros(9)
        self.positions = []

        self.scanner = Scanner(self.sample_rate)
        self.INS = INS(self.config, adpt_flag=True)
        self.callback = XdaCallback()
        self.running = False
        self.device = None

        self.init_ui()
    
    @staticmethod
    def load_config(file_path):
        return OmegaConf.load(file_path)
 
    def reset_initial_position(self):
        self.position = np.zeros(3)
        self.state_vector = np.zeros(9)
    
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

    def connect_sensor(self):
        # open port
        self.device = self.scanner.scan_and_open() 

        # Display sensor information
        device_info = self.scanner.get_device_info()
        info_str = ', '.join([f"{key}: {value}" for key, value in device_info.items()])
        self.info_label.config(text=f"Sensor Information: {info_str}")

        self.connect_button.config(state=tk.DISABLED)
        self.disconnect_button.config(state=tk.NORMAL)
        self.start_button.config(state=tk.NORMAL)

    def disconnect_sensor(self):
        if self.device:
            self.device.stopRecording()
            self.scanner.close()
        self.connect_button.config(state=tk.NORMAL)
        self.disconnect_button.config(state=tk.DISABLED)
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.DISABLED)
        self.reset_initial_position()

        self.info_label.config(text="Sensor Information: Not Connected")
    
    def collect_data(self):
        while self.running:
            window_size = int(self.time_window * self.sample_rate)
            if self.callback.packetAvailable():
                s = ""
                packet = self.callback.getNextPacket()
                u = self.process_data(packet)
                s += "Acc X: %.2f" % u[0] + ", Acc Y: %.2f" % u[1] + ", Acc Z: %.2f" % u[2]
                s += "Gyro X: %.2f" % u[3] + ", Gyro Y: %.2f" % u[4] + ", Gyro Z: %.2f" % u[5]
                self.data_label.config(text=s)
                self.data_text.insert(tk.END, s + '\n')
                self.data_text.see(tk.END)
                self.u_window.append(u)

                if len(self.u_window) > window_size:
                    us = np.array(self.u_window)
                    zupt, logL = self.INS.detector(us.T)
                    x_h, quat = self.INS.baseline(us.T, zupt, logL)
                    x, y, z = x_h[0:3, -1]
                    self.position[0] += x
                    self.position[1] += y
                    self.position[2] += z
                    # append data into class list
                    self.positions.append(self.position.tolist())
                    self.u_window.clear() # discarding the used batch
                    self.visualize_data()
                print("%s\r" % s, end="", flush=True)


    def start_collection(self):
        self.running = True
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)

        self.positions.clear()  # clear the previous collection

        self.collection_thread = threading.Thread(target=self.collect_data)
        self.collection_thread.start()

    def stop_collection(self):
        self.running = False
        if self.collection_thread.is_alive():
            self.collection_thread.join()
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)

    def process_data(self, data_packet):
        acc = data_packet.calibratedAcceleration()
        gyro = data_packet.calibratedGyroscopeData()
        return np.array([acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]])
    
    def visualize_data(self):
        if len(self.positions) > 0:
            pos = np.array(self.positions)

            self.ax_position.clear()
            self.ax_position.plot(pos[:, 0], pos[:, 1])
            max_range = np.max(np.abs(pos)) * 1.1
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

                # Update orientation visualization
                self.ax_orientation.clear()
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
            packet = self.callback.getNextPacket()
            self.visualize_orientation(packet)
        self.after(100, self.update_orientation)
   

if __name__ == "__main__":
    app = NavApp()
    app.mainloop
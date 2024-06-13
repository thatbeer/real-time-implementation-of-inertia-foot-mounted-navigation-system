import sys
import threading
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk
import xsensdeviceapi as xda
from mpl_toolkits.mplot3d import Axes3D

from omegaconf import OmegaConf

from Init_det_glrt import Init_det_glrt
from ZUPTaidedINS import ZUPTaidedINS
from detector import detector_adaptive
# from xda_utils import *
from INS import INS

from threading import Lock

class Payload:
    def __init__(self):
        self.timestamp = []
        self.acc = np.array([0, 0, 0])
        self.angular_velocity = np.array([0, 0, 0])

class XdaCallback(xda.XsCallback):
    def __init__(self, max_buffer_size=5):
        super().__init__()
        self.m_maxNumberOfPacketsInBuffer = max_buffer_size
        self.m_packetBuffer = list()
        self.m_lock = Lock()

    def packetAvailable(self):
        with self.m_lock:
            return len(self.m_packetBuffer) > 0

    def getNextPacket(self):
        with self.m_lock:
            assert len(self.m_packetBuffer) > 0
            return xda.XsDataPacket(self.m_packetBuffer.pop(0))

    def onLiveDataAvailable(self, dev, packet):
        with self.m_lock:
            assert packet is not None
            if len(self.m_packetBuffer) >= self.m_maxNumberOfPacketsInBuffer:
                self.m_packetBuffer.pop(0)
            self.m_packetBuffer.append(xda.XsDataPacket(packet))

class Scanner:
    def __init__(self, retries=3, wait_time=1):
        self.control = xda.XsControl_construct()
        assert self.control != 0
        self.device = None
        self.retries = retries
        self.wait_time = wait_time

    def scan_and_open(self):
        attempt = 0
        while attempt < self.retries:
            print("Scanning for devices... Attempt:", attempt + 1)
            portInfoArray = xda.XsScanner_scanPorts()
            
            mtPort = xda.XsPortInfo()
            for i in range(portInfoArray.size()):
                if portInfoArray[i].deviceId().isMti() or portInfoArray[i].deviceId().isMtig():
                    mtPort = portInfoArray[i]
                    break
            
            if mtPort.empty():
                print("No MTi device found. Retrying...")
                attempt += 1
                time.sleep(self.wait_time)
                continue
            
            did = mtPort.deviceId()
            print(f"Found a device with ID: {did.toXsString()}, Port: {mtPort.portName()}")

            if not self.control.openPort(mtPort.portName(), mtPort.baudrate()):
                raise RuntimeError("Could not open port. Aborting.")
            
            self.device = self.control.device(did)
            assert self.device is not None
            print(f"Device {self.device.productCode()} with ID {self.device.deviceId().toXsString()} opened.")
            return self.device
        
        raise RuntimeError("No MTi device found after multiple attempts. Aborting.")

    def get_device_info(self):
        if self.device:
            device_info = {
                "Product Code": self.device.productCode(),
                "Device ID": self.device.deviceId().toXsString(),
                "Firmware Version": self.device.firmwareVersion().toXsString()
            }
            return device_info
        return {}

    def close(self):
        if self.device:
            self.device.stopRecording()
            self.device.closeLogFile()
        if self.control:
            self.control.close()
        print("Disconnected successfully.")


def load_config(file_path):
    return OmegaConf.load(file_path)

config = load_config('config.yaml')


class NavigationApp(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("Real-Time Foot-Mounted Navigation")
        self.geometry("1200x800")

        self.config = config
        self.sampling_rate = config.general_parameters.sample_rate
        self.scanner = Scanner()
        self.callback = XdaCallback()
        # self.simdata = Init_det_glrt()
        self.INS = INS(self.config, adpt_flag=True)
        self.adpt_flag = True
        self.running = False
        self.device = None

        self.time_window = 1.0
        self.sampling_rate = 100
        self.u_window = []
        self.position = np.zeros((3, 1))
        self.state_vector = np.zeros((9, 1))
        self.state_position = []

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

    def start_collection(self):
        self.running = True
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)

        self.state_position.clear()  # clear the previous collection

        self.collection_thread = threading.Thread(target=self.collect_data)
        self.collection_thread.start()

    def stop_collection(self):
        self.running = False
        if self.collection_thread.is_alive():
            self.collection_thread.join()
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)

    def reset_initial_position(self):
        self.position = np.zeros((3, 1))
        self.state_vector = np.zeros((9, 1))

    def collect_data(self):
        while self.running:
            window_size = int(self.time_window * self.sampling_rate)
            if self.callback.packetAvailable():
                s = ""
                packet = self.callback.getNextPacket()
                u = self.process_data(packet)
                s += "Acc X: %.2f" % u[0] + ", Acc Y: %.2f" % u[1] + ", Acc Z: %.2f" % u[2]
                self.data_label.config(text=s)
                self.data_text.insert(tk.END, s + '\n')
                self.data_text.see(tk.END)  # Scroll to the end of the text box

                self.u_window.append(u)
                if len(self.u_window) > window_size:
                    us = np.array(self.u_window)
                    # zupt, logL = detector_adaptive(us.T, self.simdata)
                    # x_h, _ = ZUPTaidedINS(us.T, zupt, logL, self.adpt_flag, self.simdata)
                    zupt , logL = self.INS.detector(us.T)
                    x_h, _ = self.INS.baseline(us.T, zupt, logL)
                    x, y, z = x_h[0:3, -1]
                    self.position[0] += x
                    self.position[1] += y
                    self.position[2] += z
                    self.state_position.append(self.position.ravel().tolist())
                    self.u_window.clear()
                    self.visualize_data()
                print("%s\r" % s, end="", flush=True)
            else:
                time.sleep(0.01)  # Wait briefly if no packet is available

    def process_data(self, data_packet):
        acc = data_packet.calibratedAcceleration()
        gyro = data_packet.calibratedGyroscopeData()
        return np.array([acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]])

    def visualize_data(self):
        if len(self.state_position) > 0:
            pos = np.array(self.state_position)

            self.ax_position.clear()
            self.ax_position.plot(pos[:, 0], pos[:, 1])
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
            packet = self.callback.getNextPacket()
            self.visualize_orientation(packet)
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

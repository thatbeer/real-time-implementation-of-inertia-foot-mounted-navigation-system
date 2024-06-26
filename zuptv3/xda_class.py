import time
import numpy as np
from threading import Lock
import xsensdeviceapi as xda



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
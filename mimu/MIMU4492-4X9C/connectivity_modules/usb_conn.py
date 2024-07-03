# Implemented class USB serial port for ble connection
# Implemented important methods for communication with devia via serial port

#     Date     |    Review    |      Author                      |  Comments
# -------------+--------------+-----------------------------------------------------
#   13-06-2018 |    0.0       |   Ajit Gupta                   | Implemented methods for serial communication

import glob
import serial
import sys
import connectivity


class UsbConnectivity(connectivity.Connectivity):

    def __init__(self):
        self.usb_serial = None
        super(UsbConnectivity, self).__init__()

    def open(self, device_params):
        """
            Connect to USB port which connect to device
        :param device_params: tuple contains port name(String) and baud rate(integer)
        :raises Exceptoins:
                Failed to open device
        :return: None
        """
        try:
            self.input_validation(device_params[0])
            print "Opening Serial device", device_params[0]
            self.usb_serial = serial.Serial(device_params[0], device_params[1])
        except Exception as ex:
            print "Exception on opening serial: ", ex.message
            raise ex

        return self.usb_serial

    def send(self, data):
        """
            Send the data to destination device
        :param data:
        :return: None
        """
        self.usb_serial.write(data)
        self.usb_serial.flushOutput()

    def receive(self, length):
        """
            receive the data from connected device
        :param length: size of data to receive
        :return: None
        """
        rcv_buffer = self.usb_serial.read(length)
        return rcv_buffer.encode("hex")

    def close(self):
        """
            Close the socket connection
        :returns:  None
        """
        self.usb_serial.close()

    def input_validation(self, *kargs):
        """
            Check whether port_name is exists or not and its read/write permission
        :param kargs: port name
        :raises EnvironmentError or Exception:
            On unsupported or unknown platforms
            Port name does not exist
        :returns:
            True or False
        """
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')

        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        if kargs[0] not in result:
            if sys.platform.startswith('linux') or sys.platform.startswith('cygwin') or \
                    sys.platform.startswith('darwin'):
                raise Exception(
                    "Port name does not exists. Please verify the port name and its read/write permission."
                    "\nAvailable ports:- " + str(result))
            else:
                raise Exception("Port name does not exists. Please verify the port name."
                                "\nAvailable ports:- " + str(result))
        return True

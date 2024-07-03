# Implemented class for WiFi connectivity
# Implemented important methods for communication with device through WiFi communication

#     Date     |    Review    |      Author                      |  Comments
# -------------+--------------+-----------------------------------------------------
#   13-06-2018 |    0.0       |   Ajit Gupta                   | Implemented methods for serial communication


import socket
import time
import connectivity


class WifiConnectivity(connectivity.Connectivity):
    def __init__(self):
        self.client_socket = socket.socket()
        super(WifiConnectivity, self).__init__()

    def open(self, device_params):
        """
            Create TCP socket and connect to device.
        :param device_params: tuple contains IP address(string) and port number(integer)
        :raises Exceptoins:
                Failed to open device
        :return: None
        """
        try:
            self.input_validation(device_params[0], device_params[1])
            print "Connecting to device:", device_params[0]
            self.client_socket.connect((device_params[0], device_params[1]))  # connect to the server
        except Exception as ex:
            print "Exception on socket creation: ", ex.message
            raise ex
        return self.client_socket

    def send(self, byte_buffer):
        """
            Send the data to destination device
        :param byte_buffer:
        :return: None
        """
        str_buffer = ("".join(str('{0:02x}'.format(e)) for e in byte_buffer)).decode("hex")
        # if(len(buffer) < MAX_CMD_LENGTH):
        self.client_socket.send(str_buffer)

    def receive(self, length):
        """
            receive the data from connected device
        :param length: size of data to receive
        :return: None
        """
        recv_buffer = ''
        data_len = 0
        while data_len < length:
            recv_buffer += self.client_socket.recv(length - data_len)
            data_len += len(recv_buffer)
            if data_len < length:
                time.sleep(0.005)
        return recv_buffer.encode("hex")

    def close(self):
        """
            Close the socket connection
        :returns NoneType:
                None
        """
        self.client_socket.close()

    def input_validation(self, *kargs):
        """
            Check whether IP address is valid or not
        :raises Exceptions:
            Invalid IP address or port name
        :returns Boolean:
            True or False
        """
        status = False
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2)
            result = sock.connect_ex((kargs[0], kargs[1]))
            if result == 0:
                status = True
            else:
                raise Exception("Unable to connect the device. IP("+kargs[0]+") and Port("+str(kargs[1])+")"
                                ".\nPlease verify the IP address and port num.")
            sock.close()
        except Exception as ex:
            raise ex
        return status

# Implemented class Bluetooth Low Energy(BLE) for ble connection
# Implemented important methods for communication with ble devices

#     Date     |    Review    |      Author                      |  Comments
# -------------+--------------+-----------------------------------------------------
#   13-06-2018 |    0.0       |   Ajit Gupta                   | Implemented methods for BLE connectivity
import time
from Queue import Queue
import pexpect
import re
import thread
import connectivity


class BleConnectivity(connectivity.Connectivity):

    def __init__(self):
        self.gatt = None
        self.timeout = 0.002
        self.ble_buffer = ''
        self.final_str_data = ''
        self.queue = Queue()
        self.isRunning = True
        super(BleConnectivity, self).__init__()

    def open(self, device_params):
        """
            Using pexpect module, run the gatttool utilities which use for communication with BLE device
        :param device_params: tuple contains MAC address
        :raises Exceptoins:
                Failed to open device
        :return NoneType: None
        """
        print "Opening Serial device", device_params[0]
        try:
            self.input_validation(device_params[0])
            # Connect to the device.
            self.gatt = pexpect.spawn('gatttool -I')
            self.gatt.sendline('connect {0}'.format(device_params[0]))
            self.gatt.expect('Connection successful')
            self.gatt.sendline('mtu 512')
            self.gatt.expect('MTU was exchanged successfully:')
            self.gatt.sendline('char-write-req 0x000f 01')
            self.gatt.expect('Characteristic value was written successfully')
            # gatt.expect("\r\n", timeout=10)
            thread.start_new_thread(self.read_realtime, ())
        except Exception as e:
            print "Exception on connecting to device: ", e.message
            raise e
        return self.gatt

    @staticmethod
    def bytebuffer_to_str(byte_buffer):
        """
            Convert list of bytes to string format
        :param byte_buffer:
        :return NoneType: String
        """
        bytes_string = map('{:02x}'.format, byte_buffer)
        str_buffer = ''.join(bytes_string).upper()
        return str_buffer

    def send(self, byte_buffer):
        """
            Send the data to destination device
        :param byte_buffer:
        :return NoneType: None
        """
        cmd = self.bytebuffer_to_str(byte_buffer)
        self.gatt.sendline('char-write-cmd 0x0011 {0}'.format(cmd))
        self.gatt.expect('')

    def receive(self, length):
        """
            receive the data from connected device
        :param length: size of data to receive
        :return NoneType: None
        """
        strg = ''
        length = length * 2
        while not self.queue.empty or len(self.ble_buffer) < length:
            data_frm_queue = self.queue.get()
            # print data_frm_queue
            self.ble_buffer += data_frm_queue
            self.queue.task_done()
            time.sleep(0.004)
        if len(self.ble_buffer) >= length:
            strg = self.ble_buffer[:length]
            self.ble_buffer = self.ble_buffer[length:]
        return strg

    def read_realtime(self):

        try:
            while self.isRunning:
                pkt_buffer = ''
                i = self.gatt.expect([pexpect.TIMEOUT], timeout=self.timeout)
                gatt_str = ''
                if i == 0:
                    gatt_str = self.gatt.before
                    if self.gatt.before:
                        self.gatt.expect('.+')
                pkt_list = gatt_str.strip().splitlines()
                for pkt in pkt_list:
                    pkt = pkt.strip()
                    if ">" not in pkt and len(pkt) > 0:
                        pkt = pkt.replace("Notification handle = 0x000e value:", "")
                        pkt = pkt.replace("\x1b[", "")
                        pkt = pkt.replace("K", "")
                        pkt = pkt.replace(" ", "")
                        pkt = pkt.replace('Characteristicvaluewaswrittensuccessfully', "")
                        pkt = pkt.replace('char-write-req0x000f01', '')
                        pkt = pkt.replace('char-write-cmd0x0011', '')
                        pkt = pkt.replace('char-write-cmd', '')
                        pkt = pkt.replace('a04000e0', "")
                        # pkt = pkt.replace('char-writ', "")
                        # pkt = pkt.replace('e-cmd0x0011', "")
                        # pkt = pkt.replace('0x001', "")
                        # pkt = pkt.replace('char-writ', "")
                        # print pkt
                        # pkt_buffer += pkt
                        self.queue.put(pkt)
        except Exception as e:
            print "ERROR :", e

    def close(self):
        """
            Close the connection
        :returns NoneType:  None
        """
        self.isRunning = False
        self.gatt.sendline('disconnect')

    def input_validation(self, *kargs):
        """
            Check whether MAC address is valid or not
        :param kargs: port name
        :raises Exceptions:
            Invalid MAC address
        :returns Boolean:
            True or False
        """
        if re.match("[0-9a-f]{2}([-:]?)[0-9a-f]{2}(\\1[0-9a-f]{2}){4}$", kargs[0].lower()):
            return True
        else:
            raise Exception("Invalid MAC address")

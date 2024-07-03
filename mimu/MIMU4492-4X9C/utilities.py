import random
import string
import struct
import tkMessageBox
from Tkinter import *
import serial
import time
import pyqtgraph as pg
from connectivity_modules.wifi_conn import WifiConnectivity
from connectivity_modules.ble_conn import BleConnectivity
from connectivity_modules.usb_conn import UsbConnectivity


pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')
isRunning = True

plot_descs = {
    1: ["ACC", 'a', '[m/s^2]'],
    2: ["GYRO", "\omega", ' [deg/s]'],
    3: ["MAG", "tesla", ' [mictes]']
}
app = None


def get_connectivity(c_type):
    if str(c_type).upper() == 'WIFI':
        return WifiConnectivity()
    elif str(c_type).upper() == 'BLE':
        return BleConnectivity()
    elif str(c_type).upper() == 'USB':
        return UsbConnectivity()

    return None


def get_file_value(filename):  # to read value from a file
    filevalue = file(filename, 'r')
    value = filevalue.read()
    filevalue.close()
    return int(value)


def get_y_value(gvalue):  # to get thr y values from the file
    y = []
    yvalue = file('data/yaxis.txt', 'r')
    for i in range(0, 4):
        y.append(float(yvalue.readline()) * gvalue)
    return y


def get_rate_divider(a):  # to get the rate divider value
    if int(a) == 1:
        return 0
    else:
        return 1 + get_rate_divider(a / 2)


def get_checksum(pkt):  # returns the checksum of packet
    return struct.unpack("!H", pkt[-2:])[0]


def cal_checksum(pkt):  # returns the calculated checksum of packet
    checksum = 0
    a = pkt[:-2].encode('hex')
    x = 0
    y = 2
    for i in range(0, len(a)/2):
        checksum += int(a[x:y], 16)
        x += 2
        y += 2
    return checksum

# Convert list of byte to string e.g. [1,4] ==> '0000000100000100'


def convert_bytes_to_string(byte_array):
    binary_str = ''
    for byte in byte_array:
        binary_str += '{0:08b}'.format(byte)
    return binary_str


def id_generator(size=6, chars=string.ascii_lowercase + string.digits):
    return ''.join(random.choice(chars) for _ in range(size))


def create_logfile(binary_string, select_acm, dlogfile):  # create log file and add header base on selection
    filedata = open(dlogfile, "w")
    # str_fmt = "%11s\t" + "%10s\t"
    str_fmt = "%11s\t"
    data_fmt = ["Packet No."]
    for indx in range(0, len(binary_string)):
        if binary_string[indx] == '1':
            acc_gyro = select_acm[indx]
            for j in range(0, len(acc_gyro)):
                if acc_gyro[j] == '1':
                    axis = j % 3
                    str_fmt += "%10s\t"
                    if j < 3:
                        data_fmt.append("a" + str(axis) + str(indx)+" m/s^2")
                    elif j < 6:
                        data_fmt.append("g" + str(axis) + str(indx) + " deg/s")
                    else:
                        data_fmt.append("m" + str(axis) + str(indx) + " mtesla")

    str_fmt = str_fmt.strip() + "\n"
    # print str_fmt
    # print data_fmt
    hdr_str = str_fmt % tuple(data_fmt)
    filedata.write(hdr_str)
    return filedata

def get_plot_options(select_acm, binary_string):
    """
     Get Plot options
     :param select_acm:
     :param binary_string:
     :return list of number e.g. [1, 2 , 3 ] mean Plot must
            contains Acc(1), Gyro (2) and Magn(3):
    """
    plot_opt = set()
    for indx in range(0, len(binary_string)):
        if binary_string[indx] == '1':
            acc_gyro = select_acm[indx]
            for j in range(0, len(acc_gyro)):
                if acc_gyro[j] == '1':
                    if j < 3:
                        plot_opt.add(1)
                    elif j < 6:
                        plot_opt.add(2)
                    else:
                        plot_opt.add(3)
    return list(plot_opt)


def cal_checksum_cmd(cmd):
    checksum = 0
    for i in range(0, len(cmd)):
        checksum += cmd[i]
    return int(checksum / 256), checksum % 256


def open_device(device, rate):  # to open a com port
    try:
        btserial = serial.Serial(device, rate)
    except serial.SerialException as e:  # if it gives error then close the program
        root = Tk()
        root.withdraw()
        tkMessageBox.showerror("Error !", "%s\n\n Please restart Com port and the deivce" % e.message)
        stop = file("error", 'w')
        stop.close()
        sys.exit(1)
    return btserial


def get_ratedevider(a):  # to get rate devider value
    if int(a) == 1:
        return 0
    else:
        return 1 + get_ratedevider(a/2)

def write_device(device, buffer_c):  # to write commnads in the com port for the sensor
    device.write(buffer_c)
    device.flushOutput()

def read_device(device, data_len):  # to read the packets coming from the sensor
    buffer_r = device.read(data_len)
    return buffer_r

def is_hex(s):
    hex_digits = set(string.hexdigits)
    # if s is long, then it is faster to check against a set
    return all(c in hex_digits for c in s)

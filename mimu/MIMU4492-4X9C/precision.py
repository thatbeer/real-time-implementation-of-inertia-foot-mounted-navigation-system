# In Precision IMU the fused and calibrated values come from the sensor. The values
# are stored in different different lists. Using these scripts graphs are plotted.
#
# The input comes from the main script based on that real time processing takes place

# in this script.
#
#
#     Date     |    Review    |      Author      |  Comments
# -------------+--------------+------------------+----------------
#   10-07-2016 |    0.0       |   Rahul Tiwari   | Initial Release
#   21-08-2016 |    0.1       |   jaya sandhya M | included checksum check           |              |                  |
#   18-08-2018 |    0.1       |   Ajit/Vijay     | Modified code for MIMU4844 sensor
#
# (c) All Rights Reserved, GT Silicon Pvt Ltd, 2016-2018

import Queue
import os.path
import thread
from collections import deque
import binascii
from pyqtgraph.Qt import QtGui
from utilities import *

MAX_FREQ = 500  # Max data rate transmission
g_value = 9.81  # gravity value set
d_graph = 'off line'  # Display graph  #'Real Time' or 'Off Line'
log_data = 1  # Save lag data in file write 1 else 0
plot_graph = 1    # Whether graph is required or not 1 else 0
dlog_file = 'data/bin.txt'  # Data log path directory
out_rate = float(500)       # Data rate (HZ) Note: Maximum supported data rate is 500 Hz when plot_graph is set to 0,
                            # i.e. when output data is logged in file only, without any plot
# conn_params = ('00:A0:50:E4:91:98', 115200)                        # Write hare serial port on your device
# conn_type = 'ble'
conn_params = ('COM3', 460800)  # Write hare serial port on your device
conn_type = 'usb'

NUM_AXIS = 9  # number of axis e.g ax, ay, az, gx, gy, gz , mx, my and mz
FUSE_ITEM = 1  # number of fused item e.g. timestamp
select_acm = '111111111'
runtime = float(10)  # Enter time in second
binary_string = '1'
num_of_selected_imu = 1
app = None

def get_inertial_data(pkt_d):  # returns the inertial data of packet
    return struct.unpack('!I9f', pkt_d)

def insert_data_logfile(file_data, queue_d, num_imu, imu_item, fuse_item, temp_file_t, select_acm_t, binary_string_t):
    # insert data log file and temporary file, this method
    # is run in background
    print "start"
    tmp_file_fmt = "%11d\t" + "%12.3f\t" * (imu_item * num_imu + fuse_item)
    tmp_file_fmt = tmp_file_fmt.strip() + "\n"

    str_fmt = "%11d\t" + "%12.3f\t"

    # str_fmt = str_fmt.strip() + "\n"

    while isRunning:
        while not queue_d.empty():
            data_frm_queue = queue_d.get()
            # print data_frm_queue
            data_str = tmp_file_fmt % tuple(data_frm_queue)
            temp_file_t.write(data_str)  # write data to temporary file
            temp_file_t.flush()
            new_data = []
            ''' Write selectively data to output file - Start  '''
            new_data.append(data_frm_queue[0])  # pkt number
            new_data.append(data_frm_queue[1])  # time (s)
            del data_frm_queue[0]
            del data_frm_queue[0]

            new_str_fmt = str_fmt
            icu_cntry = 0
            for i in range(0, len(binary_string_t)):
                if binary_string_t[i] == '1':
                    ac_gy_ma = select_acm_t
                    acm_len = NUM_AXIS
                    for j in range(0, acm_len):
                        if ac_gy_ma[j] == '1':
                            new_data.append(data_frm_queue[acm_len * icu_cntry + j])
                            new_str_fmt += "%10.3f\t"
                    icu_cntry += 1
            new_str_fmt = new_str_fmt.strip() + "\n"

            data_str = new_str_fmt % tuple(new_data)
            # print hdr_str
            file_data.write(data_str)
            file_data.flush()
            '''   Write selectively data to output file - End   '''
        time.sleep(0.5)

def get_plot_options(select_acm_p, binary_string_p):
    """
    Get Plot options
    :param select_acm_p
    :param binary_string_p:
    :return list of number e.g. [1, 2 , 3 ] mean Plot must
            contains Acc(1), Gyro (2) and Magn(3):
    """
    plot_opt = set()
    for index in range(0, len(binary_string_p)):
        if binary_string_p[index] == '1':
            acc_gyro = select_acm_p
            for j in range(0, len(acc_gyro)):
                if acc_gyro[j] == '1':
                    if j < 3:
                        plot_opt.add(1)
                    elif j < 6:
                        plot_opt.add(2)
                    else:
                        plot_opt.add(3)
    return list(plot_opt)


screen_width = int(1600)
screen_height = int(700)

width = screen_width - 620
height = screen_height - 80

if plot_graph == 0 :
    d_graph = 'No Plot'

d_graph = d_graph.upper()

# **** making empty lists to store values ****
pkt_number = []
time_stamps = []

if d_graph == 'OFF LINE' and log_data != 1:
    log_data = 1

if os.path.isfile("stop"):
    os.remove("stop")

connectivity_obj = get_connectivity(conn_type)

if connectivity_obj is None:
    tkMessageBox.showerror("Alert", "%s\nPlease give input connectivity type e.g. USB or WiFi or  BLE")
    sys.exit(1)

# Open serial port
try:
    com = connectivity_obj.open(conn_params)
except Exception as e:
    tkMessageBox.showerror("oops", "%s\nPlease restart the device and com port and try again" % e.message)
    sys.exit(1)

pkts = 0
count = 0
start = time.time()

queue = Queue.Queue()
temp_filename = None
temp_file = None
outfile = None
if log_data:
    try:
        temp_filename = 'temp_' + id_generator()
        temp_file = open(temp_filename, 'w')
        select_acm_arr = [select_acm]
        outfile = create_logfile(binary_string, select_acm_arr, dlog_file)
        thread.start_new_thread(insert_data_logfile, (outfile, queue, num_of_selected_imu, NUM_AXIS, FUSE_ITEM,
                                                      temp_file, select_acm, binary_string))
    except Exception as e:
        print e
        print "Error: unable to start thread"
        sys.exit(-1)

scale_pr_acc = 1.0
scale_pr_gyro = 57.325
scale_pr_mag = 0.6
# generating the command to start normal imu
out_rate_ = MAX_FREQ / float(out_rate)
cmd = [0x31, 0x16, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x57]
connectivity_obj.send(cmd)
hex_val = [0x01, 0x02, 0x3, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f]
out_rate_cmd = hex_val[get_ratedevider(out_rate_)]
checksum = 0x21 + 0x08 + 0x13 + 0xA4 + out_rate_cmd
cmd = [0x21, 0x08, 0x13, 0xA4, 0x00, 0x00, 0x00, 0x00, 0x00, out_rate_cmd, 0x00, checksum]
print "cmd", cmd
connectivity_obj.send(cmd)
# *** receiving acknowledgement ***
ack = connectivity_obj.receive(8)
print "ack", ack

''' RealTime Data - Start '''
all_line = []
pXX_acm = []
all_imu_data = []
for i_imu in range(0, num_of_selected_imu * NUM_AXIS):
    all_imu_data.append([])

plot_option = get_plot_options(select_acm, binary_string)

if log_data == 0 :
    d_graph = 'No Plot'

if d_graph == 'OFF LINE' and log_data != 1:
    log_data = 1

if d_graph == 'REAL TIME':
    x_axis_value = int(int(get_file_value('data/xaxis.txt')) * float(out_rate))
    # *** queues for real time graph plotting ***
    app = QtGui.QApplication([])
    mw = QtGui.QMainWindow()
    win = pg.GraphicsWindow(title="Raw Data")
    mw.resize(width, height)
    cw = QtGui.QWidget()
    mw.setCentralWidget(cw)
    g = QtGui.QVBoxLayout()
    cw.setLayout(g)
    mw.setWindowTitle('pyqtgraph example: Plotting')

    list_pt_graph = []
    num_plot = 0
    for options in plot_option:
        # create subplot for accelerometer , gyroscope and magnetometer if selected
        if num_plot > 0:
            win.nextRow()
        num_plot += 1

        pt_desc = plot_descs[options]
        pt_graph = win.addPlot(title=pt_desc[0])
        # p1.setYRange(-(X_AXIS), (X_AXIS), padding=0.05)
        pt_graph.setLabel('left', pt_desc[1], units=pt_desc[2])
        pt_graph.showGrid(x=True, y=True)
        list_pt_graph.append(pt_graph)

    list_pt_graph[-1].setLabel('bottom', 'Timespan : %d' % int(get_file_value('data/xaxis.txt')), units='seconds')

    for i_imu in range(0, num_of_selected_imu * NUM_AXIS):
        all_line.append([])
        pXX_acm.append(None)

    s_cntry = 0
    color = ['b', 'r', 'g']  # blue for x axis, red color for y axis and green color for z axis
    # for s_imu in range(0, len(binary_string)):  # plot each axis from each imu if selected
    #     if binary_string[s_imu] == '1':
    acc_gyro_mag = select_acm
    for s_a_g in range(0, NUM_AXIS):
        plt_indx = s_cntry * NUM_AXIS + s_a_g
        all_line[plt_indx] = None
        if acc_gyro_mag[s_a_g] == '0':
            continue
        all_line[plt_indx] = deque([0] * x_axis_value, x_axis_value)
        which_colour = s_a_g % 3
        which_plt = int(s_a_g / 3)
        try:
            pXX_acm[plt_indx] = list_pt_graph[which_plt].plot(all_imu_data[plt_indx], pen=color[which_colour])
        except IndexError as e:
            # print e
            pXX_acm[plt_indx] = list_pt_graph[which_plt - 2].plot(all_imu_data[plt_indx], pen=color[which_colour])
    s_cntry += 1

''' RealTime Data - End '''
s1 = ''
pkt_size = 46
Data_in = connectivity_obj.receive(pkt_size)

while time.time() - start < runtime:  # to run the process for the run time
    if not os.path.isfile("stop"):
        try:
            Data_in = binascii.unhexlify(Data_in)
            s1 = Data_in.encode("hex")
            print time.time() - start
            # print "data_in",s1
            (start_code, pkt_num, payload_length) = struct.unpack("!BHB", Data_in[0:4])
            if struct.unpack("!B", Data_in[0])[0] == 0xAA and get_checksum(Data_in) == cal_checksum(Data_in):
                values = get_inertial_data(Data_in[4:-2])
                pkt_number.append(pkt_num)
                time_stamps.append(format((values[0] / 64e6), '.4f'))
                # time1 = time.time()
                # putting the data into lists
                data = [pkt_num, round(values[0] / 64e6, 2), round(values[1]*scale_pr_acc, 3),
                        round(values[2]*scale_pr_acc, 3), round(values[3]*scale_pr_acc, 3),
                        round(values[4]*scale_pr_gyro, 3), round(values[5]*scale_pr_gyro, 3),
                        round(values[6]*scale_pr_gyro, 3), round(values[7]*scale_pr_mag, 3),
                        round(values[8]*scale_pr_mag, 3), round(values[9]*scale_pr_mag, 3)]

                if d_graph == 'REAL TIME':
                    s_cntry = 0
                    for s_imu in range(0, len(binary_string)):  # plot each axis from each imu if selected
                        if binary_string[s_imu] == '1':
                            acc_gyro_mag = select_acm
                            for s_a_g in range(0, NUM_AXIS):
                                if acc_gyro_mag[s_a_g] == '0':
                                    continue
                                which_axis = s_a_g % 3
                                which_plt = int(s_a_g / 3)
                                plt_indx = s_cntry * NUM_AXIS + s_a_g
                                all_line[plt_indx].appendleft(data[2 + plt_indx])
                                all_line[plt_indx].pop()
                                pXX_acm[plt_indx].setData(all_line[plt_indx])
                            s_cntry += 1
                    app.processEvents()

                if log_data:
                    queue.put(data)

                Data_in = connectivity_obj.receive(pkt_size)
                pkts += 1
                count = 0

            elif re.search(b'[\d|\w]+aa.*', s1):  # search and find new packet
                lst = re.findall(b'(aa.*)', s1)
                str_rem = lst[0]
                length = len(str_rem) / 2
                pkt_rem = Data_in[-length:]
                new_len = pkt_size - length
                Data_in = connectivity_obj.receive(new_len)
                Data_in = pkt_rem.encode("hex") + Data_in
                # print "search", Data_in

            else:
                Data_in = connectivity_obj.receive(pkt_size)
                # exit the code if the packet is detecting wrong continuously for more than 5 times
                count += 1
                if count > 5:
                    count = 0

                    tkMessageBox.showinfo("Oops",
                                          "Something went wrong please restart the device and run the process again !")

                    com.close()
                    sys.exit(1)
        except TypeError as e:
            print e.message
        except KeyboardInterrupt:
            print "Error"
            cmd = [0x32, 0x00, 0x32]
            connectivity_obj.send(cmd)
            cmd = [0x22, 0x00, 0x22]
            connectivity_obj.send(cmd)
            connectivity_obj.close()
            sys.exit(1)
    else:
        connectivity_obj.send([0x32, 0x00, 0x32])
        connectivity_obj.send([0x22, 0x00, 0x22])
        com.close()
        break

try:  # exit for when run time is over
    connectivity_obj.send([0x32, 0x00, 0x32])
    connectivity_obj.send([0x22, 0x00, 0x22])
    # com.close()
except Exception:
    pass

isRunning = False

if os.path.isfile("stop"):
    os.remove("stop")

time.sleep(1)
if log_data:
    if outfile:
        outfile.close()
    if temp_file:
        temp_file.close()

stop = time.time()
time_taken = stop - start
pkt_log = 'xyz'
if d_graph == "OFF LINE" and len(pkt_number):  # to display the graph off line
    log_file = open(temp_filename)
    all_imu_data = []
    for i_imu in range(0, num_of_selected_imu * NUM_AXIS):
        all_imu_data.append([])
    while pkt_log != '':
        pkt_log = log_file.readline()
        # print pkt_log
        cols = pkt_log.split()
        if len(cols) == 0:
            break

        counter = 2
        for col_cnter in range(2, len(cols)):
            all_imu_data[col_cnter - 2].append(
                float(cols[col_cnter].strip()))  # store all data to list of lists(9 axis)

    log_file.close()

    app = QtGui.QApplication([])
    win = pg.GraphicsWindow(title="Precision IMUs")
    win.resize(1000, 600)
    win.setWindowTitle('Precision IMUs: Plotting')

    # lst_pt_graph = None
    list_pt_graph = []
    num_plot = 0
    for options in plot_option:  # create subplot for acclerometer , gyroscope and magnometer if selected
        if num_plot > 0:
            win.nextRow()
        num_plot += 1

        pt_desc = plot_descs[options]
        pt_graph = win.addPlot(title=pt_desc[0])
        # p1.setYRange(-(X_AXIS), (X_AXIS), padding=0.05)
        pt_graph.setLabel('left', pt_desc[1], units=pt_desc[2])
        pt_graph.showGrid(x=True, y=True)
        list_pt_graph.append(pt_graph)

    # list_pt_graph[-1].setLabel('bottom', 'Timespan : %d' % time_taken, units='seconds')
    list_pt_graph[-1].setLabel('bottom', 'Timespan : %d seconds' % time_taken)


    s_cntry = 0
    color = ['b', 'r', 'g']  # blue for x axis, red color for y axis and green color for z axis
    for s_imu in range(0, len(binary_string)):  # plot each axis from each imu if selected
        if binary_string[s_imu] == '1':
            acc_gyro_mag = select_acm
            # len_agm = len(acc_gyro_mag)
            for s_a_g in range(0, NUM_AXIS):
                if acc_gyro_mag[s_a_g] == '0':
                    continue
                which_colour = s_a_g % 3
                which_plt = int(s_a_g / 3)
                try:
                    list_pt_graph[which_plt].plot(all_imu_data[s_cntry * NUM_AXIS + s_a_g], pen=color[which_colour])
                except IndexError as e:
                    # print e
                    list_pt_graph[which_plt - 2].plot(all_imu_data[s_cntry * NUM_AXIS + s_a_g], pen=color[which_colour])
            s_cntry += 1

if len(pkt_number) > 0:  # to show dialog on completion
    root = Tk()
    root.withdraw()
    if d_graph:
        tkMessageBox.showinfo("Done", "Total %d packets were received in %.2f seconds" % (pkts, time_taken))
    else:
        tkMessageBox.showinfo("Done", "Data has been written in %s .\n Total %d packets were received in %.2f seconds" %
                              (dlog_file, pkts, time_taken))

if temp_filename and os.path.isfile(temp_filename):
    os.remove(temp_filename)

if app:
    sys.exit(app.exec_())

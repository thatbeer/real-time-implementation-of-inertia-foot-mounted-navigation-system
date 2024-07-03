# Raw Data is sensors ucalibrated data. It gives uncalibrated data of
# each IMU. The input for processing comes from the main script. The
# Raw Data packets are of 58 bytes containing 2 bytes eaxh for ax, ay,
# az, gx, gy, gz, for each IMU. Time stamp for 4 bytes, start code of 1
# byte,  packet number of 2 bytes payload length for 1 byte and checksum
# for 2 bytes.
#
#     Date     |    Review    |      Author      |  Comments
# -------------+--------------+------------------+----------------
#   10-07-2016 |    0.0       |   Rahul Tiwari   | Initial Release
#   21-08-2016 |    0.1       |   jaya sandhya M | included checksum check
#   18-08-2018 |    0.1       |   Ajit/Vijay     | Modified code for MIMU4844 sensor
#
# (c) All Rights Reserved, GT Silicon Pvt Ltd, 2016-2018
import Queue
import binascii
import os.path
import thread
from PyQt4 import QtGui
from collections import deque

from utilities import *

MAX_FREQ = 500                                  # Max data rate transmission
g_value = 9.81                                   # gravity value set
d_graph = 'real time'                       # Display graph  #'Real Time'#'Off Line'
log_data = 1                                   # Save lag data in file write 1 else 0
dlog_file = 'data/bin.txt'                       # Data log path directory
out_rate = float(3.90625)                            # Data rate (HZ) Note: Maximum supported data rate is 250 Hz when plot_graph is set to 0,
                                                 # i.e. when output data is logged in file only, without any plot
conn_params = ('COM3', 460800)                        # Write hare serial port on your device
conn_type = 'usb'
plot_graph = 1                                  # Whether graph is required or not 1 else 0
# conn_params = ('00:A0:50:E4:C5:8B', 115200)                        # Write hare serial port on your device
# conn_type = 'ble'

NUM_AXIS = 9                                    # number of axis e.g ax, ay, az, gx, gy, gz , mx, my and mz
FUSE_ITEM = 1                                   # number of fused item e.g. timestamp
pi = [255, 255, 255, 255]                        # select imu
# select acc, gyro, and mag [a0i a1i a2i g0i g1i g2i m0i m1i m2i] for each imu
select_acm = ['111111111', '111111111', '111111111', '111111111', '111111111', '111111111', '111111111', '111111111',
              '111111111', '111111111', '111111111', '111111111', '111111111', '111111111', '111111111', '111111111',
              '111111111', '111111111', '111111111', '111111111', '111111111', '111111111', '111111111', '111111111',
              '111111111', '111111111', '111111111', '111111111', '111111111', '111111111', '111111111', '111111111']
runtime = float(30)                              # Enter time in second

width = int(1600-500)
height = int(900-200)
app = None

isThreadRunning = False
isComplete = False
def getData(pkt_data, num_imu, no_log):
    (start_code_q, pkt_num_q, payload_length_q) = struct.unpack("!BHB", pkt_data[0:4])
    pay_val = get_payload(pkt_data[8:pkt_size - 2], num_imu)
    # time_stamps.append(format(float(time_stamp / 64e6), '.4f'))
    data_ls = [pkt_num_q]
    for i in range(0, num_imu):
        data_ls.append(round(pay_val[6 * i + 0] * scale_acc, 3))
        data_ls.append(round(pay_val[6 * i + 1] * scale_acc, 3))
        data_ls.append(round(pay_val[6 * i + 2] * scale_acc, 3))
        data_ls.append(round(pay_val[6 * i + 3] * scale_gyro, 3))
        data_ls.append(round(pay_val[6 * i + 4] * scale_gyro, 3))
        data_ls.append(round(pay_val[6 * i + 5] * scale_gyro, 3))
        k = 6 * num_of_selected_imu + 3 * i
        data_ls.append(round(pay_val[k + 0] * scale_magn, 3))
        data_ls.append(round(pay_val[k + 1] * scale_magn, 3))
        data_ls.append(round(pay_val[k + 2] * scale_magn, 3))
    return data_ls

def insert_data_logfile(filedata, queue, num_imu, imu_item, fuse_item, temp_file, select_acm, binary_string):
    # insert data log file and temporary file, this method
    # is run in background
    print "start logfile"
    global isThreadRunning, isRunning, scale_acc, scale_gyro, scale_magn, app

    tmp_file_fmt = "%11d\t" + "%12.3f\t"*(imu_item*num_imu)
    tmp_file_fmt = tmp_file_fmt.strip() + "\n"

    str_fmt = "%11d\t"

    # str_fmt = str_fmt.strip() + "\n"
    isThreadRunning = True
    while isRunning:
        while not queue.empty():
            pkt_frm_queue = queue.get()
            data_frm_queue = getData(pkt_frm_queue, num_imu, True)

            # print data_frm_queue
            data_str = tmp_file_fmt % tuple(data_frm_queue)
            temp_file.write(data_str)  # write data to temporary file
            temp_file.flush()

            new_data = []
            ''' Write selectively data to output file - Start  '''
            new_data.append(data_frm_queue[0])  # pkt number
            # new_data.append(data_frm_queue[1])  # time (s)
            del data_frm_queue[0]
            # del data_frm_queue[0]

            new_str_fmt = str_fmt
            imu_cntr = 0
            for i in range(0, len(binary_string)):
                if binary_string[i] == '1':
                    ac_gy_ma = select_acm[i]
                    # acm_len = len(ac_gy_ma)
                    acm_len = imu_item
                    for j in range(0, acm_len):
                        if ac_gy_ma[j] == '1':
                            new_data.append(data_frm_queue[acm_len * imu_cntr + j])
                            new_str_fmt += "%10.3f\t"
                    imu_cntr += 1
            new_str_fmt = new_str_fmt.strip() + "\n"

            data_str = new_str_fmt % tuple(new_data)
            # print hdr_str
            filedata.write(data_str)
            filedata.flush()
            '''    Write selectively data to output file - End   '''
            if not isComplete :
                time.sleep(0.01)
        time.sleep(0.5)
    isThreadRunning = False

def get_payload(payload, num_imu):  # to get the values from the paayload
    fmt = "!" + "7h"*num_imu + "3f"*num_imu + "3f"
    vals = struct.unpack(fmt, payload)
    # print vals
    vals = list(vals)

    vals = vals[:len(vals)-3]  # remove last 3 fused magnetometer

    del vals[num_imu*6:num_imu*7]  # remove temperature from each imus
    # print vals
    return tuple(vals)

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

binary_string = convert_bytes_to_string(pi)[::-1]  # reverse the returned string
num_of_selected_imu = binary_string.count('1')

pkts = 0
count = 0

start = time.time()

queue = Queue.Queue()
temp_filename = None
temp_file = None
outfile = None
if log_data:
    try:
        temp_filename = 'temp_'+id_generator()
        temp_file = open(temp_filename, 'w')
        outfile = create_logfile(binary_string, select_acm, dlog_file)
        thread.start_new_thread(insert_data_logfile, (outfile, queue, num_of_selected_imu, NUM_AXIS, FUSE_ITEM,
                                                      temp_file, select_acm, binary_string))
    except Exception as e:
        print e
        print "Error: unable to start thread"
        sys.exit(-1)

scale_acc = (1/2048.0)*9.80665
scale_gyro = 1/16.4
scale_magn = 0.6
cmd = [0x22, 0x00, 0x22]
connectivity_obj.send(cmd)
out_rate_ = MAX_FREQ / float(out_rate)
hex_val = [0x01, 0x02, 0x3, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f]
out_rate_cmd = hex_val[get_ratedevider(out_rate_)]
# out_rate_cmd += 0x40
# writing the  commands to start getting  RAW DATA
cmd = [0x30, 0x16, 0x00, 0x00, 0x46]
print "command given"
connectivity_obj.send(cmd)

cmd = [0x2B, pi[0], pi[1], pi[2], pi[3], out_rate_cmd]
# cmd = [0x21, 0x39, 0x3a, 0x3b, 0x3c, 0xb9,  0xba, 0xbb, 0xbc, out_rate_cmd]
cs = cal_checksum_cmd(cmd)
cmd.append(cs[0])
cmd.append(cs[1])
print "cmd: ", cmd
connectivity_obj.send(cmd)
print "second command given"
ack = connectivity_obj.receive(8)
print "Acknowledgement : ", ack

''' RealTime Data - Start '''
all_line = []
pXX_acm = []
all_imu_data = []
for i_imu in range(0, num_of_selected_imu*NUM_AXIS):
    all_imu_data.append([])

plot_option = get_plot_options(select_acm, binary_string)

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
        pt_graph.setLabel('left', pt_desc[1], units=pt_desc[2])
        pt_graph.showGrid(x=True, y=True)
        list_pt_graph.append(pt_graph)

    list_pt_graph[-1].setLabel('bottom', 'TimeSpan : %d' % int(get_file_value('data/xaxis.txt')))

    for i_imu in range(0, num_of_selected_imu * NUM_AXIS):
        all_line.append([])
        pXX_acm.append(None)

    s_cntry = 0
    color = ['b', 'r', 'g']  # blue for x axis, red color for y axis and green color for z axis
    for s_imu in range(0, len(binary_string)):  # plot each axis from each imu if selected
        if binary_string[s_imu] == '1':
            acc_gyro_mag = select_acm[s_imu]
            for s_a_g in range(0, NUM_AXIS):
                plt_index = s_cntry * NUM_AXIS + s_a_g
                all_line[plt_index] = None
                if acc_gyro_mag[s_a_g] == '0':
                    continue
                all_line[plt_index] = deque([0] * x_axis_value, x_axis_value)
                which_colour = s_a_g % 3
                which_plt = int(s_a_g/3)
                try:
                    pXX_acm[plt_index] = list_pt_graph[which_plt].plot(all_imu_data[plt_index], pen=color[which_colour])
                except IndexError as e:
                    # print e
                    pXX_acm[plt_index] = list_pt_graph[which_plt - 1].plot(all_imu_data[plt_index],
                                                                           pen=color[which_colour])
            s_cntry += 1


''' RealTime Data - End '''
s1 = ''
pkt_size = 4+4+(num_of_selected_imu*14)+(num_of_selected_imu*12)+12+2
pkt = connectivity_obj.receive(pkt_size)

while time.time() - start < runtime:  # to run the process for the runtime
    # print time.time() - start
    if not os.path.isfile("stop"):
        try:
            pkt = binascii.unhexlify(pkt)
            s1 = pkt.encode("hex")
            (start_code, pkt_num, payload_length) = struct.unpack("!BHB", pkt[0:4])
            if start_code == 0xAA and get_checksum(pkt) == cal_checksum(pkt) % 65536:
                if log_data == 1:
                    queue.put(pkt)

                if d_graph == 'REAL TIME':
                    data = getData(pkt, num_of_selected_imu, False)
                    s_cntry = 0
                    for s_imu in range(0, len(binary_string)):  # plot each axis from each imu if selected
                        if binary_string[s_imu] == '1':
                            acc_gyro_mag = select_acm[s_imu]
                            for s_a_g in range(0, NUM_AXIS):
                                if acc_gyro_mag[s_a_g] == '0':
                                    continue
                                which_axis = s_a_g % 3
                                which_plt = int(s_a_g / 3)
                                plt_index = s_cntry * NUM_AXIS + s_a_g
                                all_line[plt_index].appendleft(data[1 + plt_index])
                                all_line[plt_index].pop()
                                pXX_acm[plt_index].setData(all_line[plt_index])
                            s_cntry += 1
                    app.processEvents()

                pkts += 1
                pkt = connectivity_obj.receive(pkt_size)
                count = 0
            elif re.search(b'[\d|\w]+aa.*', s1):  # search and find new packet
                lst = re.findall(b'(aa.*)', s1)
                str_rem = lst[0]
                if len(str_rem) % 2 == 0:
                    length = len(str_rem) / 2
                else:
                    str_rem = str_rem[:-1]
                    length = len(str_rem) / 2
                pkt_rem = pkt[-length:]
                new_len = pkt_size - length
                pkt = connectivity_obj.receive(new_len)
                pkt = pkt_rem.encode('hex') + pkt
                # print "s1-pkt ",pkt
            else:
                pkt = connectivity_obj.receive(pkt_size)
                # exit the code if the packet is detecting wrong continuously for more than 5 times
                print "counter ", count
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
        # Command to stop getting RAW DATA
        cmd = [0x22, 0x00, 0x22]
        connectivity_obj.send(cmd)
        # command to stop all outputs
        print "about to close"
        com.close()
        break

isComplete = True
try:
    cmd = [0x22, 0x00, 0x22]
    connectivity_obj.send(cmd)
    # command to stop all outputs
    print "about to close"
except:
    pass

if os.path.isfile("stop"):
    os.remove("stop")

stop = time.time()
time_taken = stop - start

time.sleep(2)
isRunning = False
while isThreadRunning:
    time.sleep(1)

if log_data:
    if outfile:
        outfile.close()
    if temp_file:
        temp_file.close()

if d_graph == "OFF LINE" and pkts > 0:  # to display the graph off line
    log_file = open(temp_filename)
    all_imu_data = []
    for i_imu in range(0, num_of_selected_imu*NUM_AXIS):
        all_imu_data.append([])
    while pkt != '':
        pkt = log_file.readline()
        cols = pkt.split()
        if len(cols) == 0:
            break

        # counter = 1
        for col_canter in range(1, len(cols)):
            all_imu_data[col_canter - 1].append(float(cols[col_canter].strip()))
            # store all data to list of lists(9 axis)

    log_file.close()

    app = QtGui.QApplication([])
    win = pg.GraphicsWindow(title="RAW Data")
    win.resize(1000, 600)
    win.setWindowTitle('RAW Data: Plotting')

    list_pt_graph = []
    num_plot = 0
    for options in plot_option:  # create subplot for accelerometer , gyroscope and manometer if selected
        if num_plot > 0:
            win.nextRow()
        num_plot += 1

        pt_desc = plot_descs[options]
        pt_graph = win.addPlot(title=pt_desc[0])
        pt_graph.setLabel('left', pt_desc[1], units=pt_desc[2])
        pt_graph.showGrid(x=True, y=True)
        list_pt_graph.append(pt_graph)

    # list_pt_graph[-1].setLabel('bottom', 'TimeSpan : %d' % time_taken, units='seconds')
    list_pt_graph[-1].setLabel('bottom', 'Timespan : %d seconds' % time_taken)

    s_cntry = 0
    color = ['b', 'r', 'g']  # blue for x axis, red color for y axis and green color for z axis
    for s_imu in range(0, len(binary_string)):  # plot each axis from each imu if selected
        if binary_string[s_imu] == '1':
            acc_gyro_mag = select_acm[s_imu]
            for s_a_g in range(0, NUM_AXIS):
                if acc_gyro_mag[s_a_g] == '0':
                    continue
                which_colour = s_a_g % 3
                which_plt = int(s_a_g/3)
                try:
                    list_pt_graph[which_plt].plot(all_imu_data[s_cntry * NUM_AXIS + s_a_g], pen=color[which_colour])
                except IndexError as e:
                    list_pt_graph[which_plt - 1].plot(all_imu_data[s_cntry * NUM_AXIS + s_a_g], pen=color[which_colour])
            s_cntry += 1


if pkts > 0:  # to show dialog on completion
    root = Tk()
    root.withdraw()
    if d_graph:
        if d_graph == 'NO PLOT':
            print "Total %d packets were received in %.2f seconds" % (pkts, time_taken)
        else:
            tkMessageBox.showinfo("Done", "Total %d packets were received in %.2f seconds" % (pkts, time_taken))
    else:
        tkMessageBox.showinfo("Done", "Data has been written in %s .\n Total %d packets were received in %.2f seconds" %
                              (dlog_file, pkts, time_taken))

if temp_filename and os.path.isfile(temp_filename):
    os.remove(temp_filename)

if app:
    sys.exit(app.exec_())

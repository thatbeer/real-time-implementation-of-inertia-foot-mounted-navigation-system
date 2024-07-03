#
# Collect stepwise data from sensor
#
#
#     Date     |    Review    |      Author      |  Comments
# -------------+--------------+------------------+----------------          |              |                  |
#   18-08-2018 |    0.1       |   Ajit/Vijay     | collect pdr data from sensor.
#
# (c) All Rights Reserved, GT Silicon Pvt Ltd, 2016-2018

import binascii
import math
import Tkinter
from pyqtgraph.Qt import QtGui
from utilities import *

# conn_params = ('00:A0:50:E4:91:98', 115200)  # Write hare serial port on your device
# conn_type = 'ble'
conn_params = ('COM5', 115200)  # Write hare serial port on your device
conn_type = 'usb'
dlog_file = 'data/bin.txt'

connectivity_obj = get_connectivity(conn_type)
if connectivity_obj is None:
    tkMessageBox.showerror("Alert", "%s\nPlease give input connectivity type e.g. USB or WiFi or  BLE")
    sys.exit(1)


def create_ack(pkt_num1, pkt_num2):
    ack = [1, pkt_num1, pkt_num2, (1 + pkt_num1 + pkt_num2) / 256, (1 + pkt_num1 + pkt_num2) % 256]
    # print "".join(str('{0:02x}'.format(e)) for e in ack)
    return ack


def stop():  # to stop pdr and change the button text
    global run
    run = 0
    pdrbtn.config(text="Start PDR!", command=start_pdr)
    pdrbtn.config()


def get_stepcount(pkt):  # to get the step count value frmo the packet
    return struct.unpack("!H", pkt[60:62])[0]


def get_data(pkt):  # to get inertial data from the payload
    return struct.unpack("!14f", pkt[4:60])


def parse_pkt(pkt):  # Parser for PDR. Returns packet info and inertial data
    data = []
    pkt_info = []
    s2 = ''
    pkt_size = 64
    count = 0
    valid = 0
    while valid == 0:
        try:
            # print "unhexlify: ",pkt
            pkt = binascii.unhexlify(pkt)
            s1 = pkt.encode("hex")
            # print "unhexlify:",s1
            (start_code, pkt_num1, pkt_num2, payload_length) = struct.unpack("!BBBB", pkt[0:4])
            if start_code == 0xAA and get_checksum(pkt) == cal_checksum(pkt):
                valid = 1
                count = 0
                step_count = get_stepcount(pkt)
                pkt_info = (pkt_num1, pkt_num2, step_count)
                data = get_data(pkt)
                # print "done"
            elif re.search(b'[\d|\w]+aa.*', s1):  # search and find new packet
                lst = re.findall(b'(aa.*)', s1)
                str_rem = lst[0]
                length = len(str_rem) / 2
                pkt_rem = pkt[-length:].encode('hex')
                new_len = 64 - length
                pkt = connectivity_obj.receive(new_len)
                pkt = pkt_rem + pkt
                # print "s1:", pkt
            else:
                pkt = connectivity_obj.receive(64)
                # print "new from unhexlify:", pkt
                # count += 1

                # exit the code if the packet is detectig wrong continuously for more than 5 times
                count += 1
                print 'exiting --more than 5 error packets %s', str(count)
                if count > 5:
                    count = 0
                    tkMessageBox.showinfo("Oops",
                                          "Something went wrong please restart the device and run the process again !")
                    sys.exit(1)
        except TypeError:
            # print e, "Data Error: ", pkt
            for chk in range(0, len(pkt)):
                if is_hex(pkt[chk]) is True:
                    s2 += pkt[chk]
            if re.search(b'[\d|\w]+aa.*', s2):  # search and find new packet
                lst = re.findall(b'[\d|\w]+(aa.*)', s2)
                str_rem = lst[0]
                length = len(str_rem) / 2
                pkt_rem = pkt[-length * 2:]
                new_len = 64 - length
                pkt = connectivity_obj.receive(new_len)
                pkt = pkt_rem + pkt
                # print "s2", pkt
            else:
                pkt = connectivity_obj.receive(pkt_size)
                # print "new packet", pkt
                # exit the code if the packet is detecting wrong continuously for more than 5 times
                count += 1
                if count > 5:
                    tkMessageBox.showinfo("Oops",
                                          "Something went wrong please restart the device and run the process again !")
                    sys.exit(1)

    return valid, pkt_info, data


x = []
y = []


def select_dim(xpos, ypos):
    x.append(float(format(xpos, '.4f')))
    y.append(float(format(ypos, '.4f')))
    p1.plot(x=x, y=y, pen='b')
    app.processEvents()


def start_pdr():  # this function will be called when start PDR button in PDR will be pressed
    outdata = []
    # write command to start dead step reckoning
    count = 0
    pkt_len = 64
    num_pkts = 0
    prev_pkt = -1
    global gatt
    xpos = 0.0  # x-coord in user's reference frame
    ypos = 0.0  # y-coord in user's reference frame
    zpos = 0.0  # z-coord in user's reference frame
    phi = 0.0  # Angular position around Z-axis in user's reference frame
    # Open serial port
    try:
        gatt = connectivity_obj.open(conn_params)
    except Exception as e:
        tkMessageBox.showerror("oops", "%s\nPlease restart the device and com port and try again" % e.message)
        sys.exit(1)
    cmd = [0x34, 0x00, 0x34]
    connectivity_obj.send(cmd)
    global run, p1, app
    ack = connectivity_obj.receive(4)
    print "cmd ack: ", ack
    app = QtGui.QApplication([])
    win = pg.GraphicsWindow()
    p1 = win.addPlot()
    p1.showGrid(x=True, y=True)
    while True:
        if run == 1:
            pdrbtn.configure(text="Stop", command=stop)
            pdrbtn.update()
            str1 = connectivity_obj.receive(64)
            # print "str1: ",str1
            valid, packet_info, data = parse_pkt(str1)
            if not valid:
                continue
            curr_pkt = packet_info[0] * 256 + packet_info[1]
            ack = create_ack(packet_info[0], packet_info[1])
            print "ack ", ack
            connectivity_obj.send(ack)
            if curr_pkt == prev_pkt:
                continue

            dx, dy, dz, dp, disp = calc_disp(data, phi)

            xpos += dx
            ypos += dy
            zpos += dz
            phi += dp

            num_pkts += 1
            count += pkt_len
            prev_pkt = curr_pkt

            outdata.append(format(xpos, '.2f'))
            outdata.append(format(ypos, '.2f'))
            outdata.append(format(zpos, '.2f'))
            outdata.append(format(phi, '.2f'))
            outdata.append(format(disp, '.2f'))
            select_dim(xpos, ypos)

        else:
            # gatt.sendline('char-write-cmd 0x0011 320032')
            cmd = [0x32, 0x00, 0x32]
            connectivity_obj.send(cmd)
            cmd = [0x22, 0x00, 0x22]
            connectivity_obj.send(cmd)
            # gatt.sendline('char-write-cmd 0x0011 220022')
            tkMessageBox.showinfo("Done !", "Process finished successfully")
            out = open(dlog_file, "wb")
            for i in range(0, (len(outdata) / 5) - 1):
                out.write("%d\t%s\t%s\t%s\t%s\t%s\n" % (
                    i + 1, outdata[i], outdata[i + 1], outdata[i + 2], outdata[i + 3], outdata[i + 4]))
                out.write("\n")
            out.flush()
            print "Written into file"
            out.close()
            connectivity_obj.close()
            if app:
                sys.exit(app.exec_())
            break
    run = 1


def calc_disp(sensor_data, theta):
    d0 = sensor_data[0]
    d1 = sensor_data[1]
    d2 = sensor_data[2]
    d3 = sensor_data[3]
    sin_theta = math.sin(theta)
    cos_theta = math.cos(theta)

    dx = d0 * cos_theta - d1 * sin_theta
    dy = d0 * sin_theta + d1 * cos_theta
    dz = d2
    dp = d3

    disp = math.sqrt(dx * dx + dy * dy + dz * dz)
    return dx, dy, dz, dp, disp


def calc_dist(xx, yy, z):
    r = xx * xx + yy * yy + z * z
    return math.sqrt(r)


window = Tkinter.Tk()
pdrbtn = Button(window, text="Start PDR !", command=start_pdr)
pdrbtn.grid(row=80, columnspan=3, pady=(200, 200))
run = 1
window.mainloop()

Please follow the instructions below to run the python.

1. Setup the python
   Follow the documents for installing python and dependecies.
	https://www.inertialelements.com/oblu/resources/python-setup-guidelines.pdf

2. Run python script as below:
   > python <file.py> ---> e.g. >python calibrated.py
	
	To change the parameter like comport, outrate etc, open the script with editor and go to top the page. For example in precision_imu_usb.py

	d_graph = 'off line'  			# Display graph  #'Real Time'#'Off Line'
	log_data = 0  					# Save lag data in file write 1 else 0
	dlog_file = 'data/bin.txt'  	# Data log path directory
	out_rate = float(250)  		    # Data rate (Hz) Note: Maximum supported data rate is 250 Hz when plot_graph is set to 0,
									# i.e. when output data is logged in file only, without any plot
	conn_params = ('COM5', 115200)  # Write here serial port or mac address on your device
	conn_type = 'usb'				# write here connectivity type e.g. usb or ble
	runtime = float(10)  			# Enter time in second
	plot_graph = 0                  # Whether graph is need to plot or not 0 or else

	# select acc, gyro, and magno [a0i a1i a2i g0i g1i g2i m0i m1i m2i] for each imu 
	select_acm = ['111111111', '111111111', '111111111', '111111111', '111111111', '111111111', '111111111', '111111111',
				  '111111111', '111111111', '111111111', '111111111', '111111111', '111111111', '111111111', '111111111',
				  '111111111', '111111111', '111111111', '111111111', '111111111', '111111111', '111111111', '111111111',
				  '111111111', '111111111', '111111111', '111111111', '111111111', '111111111', '111111111', '111111111']
			  
3) To find the serial port name , refer the documents https://www.inertialelements.com/oblu/resources/how-to-find-serial-port.pdf			  

	
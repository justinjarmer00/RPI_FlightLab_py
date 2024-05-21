#!/usr/bin/python3
import serial, string, datetime, time, Adafruit_ADS1x15, struct, adafruit_gps
import RPi.GPIO as GPIO
import numpy as np

# Wrapper function for pyserial readline function
class ReadLine:
	def __init__(self,s):
		self.buf = bytearray()
		self.s = s
		# print(s)

	def readline(self):
		# MY Readline Function
		# self.buf = bytearray()
		while True:
			i = self.buf.find(b"\n")
			if i >= 0:
				r = self.buf[:i-1]
				self.buf = self.buf[i+1:]
				if len(r) == 26:
					return r
			data = self.s.read(60) # if I always read twice the length of a frame of data, then I guarantee I will have at lease 1 full frame
			i = data.find(b"\n")
			if i >= 0: # check if I found a new line character in the bytes I read
				q = self.buf + data[:i-1] # by choosing k-1 I eliminate both \r\n bytes
				self.buf[0:] = data[i+1:]
				if len(q) == 26: # if my current byte array, 'r' is the correct length print it out
					return q
			else:
				self.buf.extend(data)

xbee = serial.Serial(
    port ='/dev/ttyS0',  #/dev/ttyS0',
    baudrate = 9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)
# ser1 = serial.Serial('/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0',115200)
# ser2 = serial.Serial('/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0',115200)
# ser3 = serial.Serial('/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0',115200)
# ser4 = serial.Serial('/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0',115200)

# Set values on ADC for alpha beta probe
adc = Adafruit_ADS1x15.ADS1115()
GAIN = 2/3

# Create object for readline wrapper function above
# r1 = ReadLine(ser1)
# r2 = ReadLine(ser2)
# r3 = ReadLine(ser3)
# r4 = ReadLine(ser4)

# Set up GPIO to send signal to NI DAQ and trigger the CVA relay
GPIO.setmode(GPIO.BCM)
GPIO.setup(24, GPIO.OUT)

# Set up GPS
GPS_serial = serial.Serial('/dev/ttySOFT0')
gps = adafruit_gps.GPS(GPS_serial, debug=False) 
gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
gps.send_command(b"PMTK220,1000")
last_print = time.monotonic()


# Ope file calibration file
current_date = input('Input current date (mm_dd_yy): ')
flight = input('Input flight number (Flight_#): ')
location = "/home/pi/Documents/data_" + current_date + "_" + flight + "/"
# filename = "CALIBRATION_" + current_date + "_" + flight + ".txt"
# calfilestring = location + filename
# calfile = open(calfilestring,'r')
# CAL_str = calfile.read().split('\t')
# CAL_str = CAL_str[0:52]
# CAL = tuple(map(float,CAL_str))


print("Waiting for start command...")
xbee.write(str.encode("Waiting for start command...\r\n"))

p_vals = [0 for i in range(52)]
pcheck = 0
while True:
	if xbee.in_waiting > 0:
		message = xbee.read(xbee.in_waiting)
		message = message.decode()
		if message == "s": # Or skip decode and check if equal to 73
			print("Read command received.")
			xbee.write(str.encode("Read command received."))
			xbee.write(str.encode('\r\n'))
		else:
			time.sleep(0.25)
			break
    
		now = datetime.datetime.now()
		current_time = now.strftime("%H_%M_%S")
		location = "/home/pi/Documents/data_" + current_date + "_" + flight + "/"
		filename = "pressure_data_" + current_time + ".txt"
		filestring = location + filename
		file = open(filestring,'w')
		
		print(filename + " successfully opened!")
		xbee.write(str.encode(filename + " successfully opened!"))
		xbee.write(str.encode('\r\n'))
		time.sleep(0.1)
		print("Reading pressure data...")
		xbee.write(str.encode("Reading pressure data..."))
		xbee.write(str.encode('\r\n'))
		
		file.write("Sensor Numbers \r\n")
		file.write("Alpha\tPitot(m/s)\tGPS(m/s)\tGPS_fix\t1\t2\t3\t4\t5\t6\t7\t8\t9\t10\t11\t12\t13\t14\t15\t16\t17\t18\t19\t20\t21\t22\t23\t24\t25\t26\t27\t28\t29\t30\t31\t32\t33\t34\t35\t36\t37\t38\t39\t40\t41\t42\t43\t44\t45\t46\t47\t48\t49\t50\t51\t52\r\n")
		
		a_b = ([0 for q in range(2)])
		while True:
			# Set GPIO high to indicate start of collection for NI DAQ
			GPIO.output(24,1)

			# Get GPS update
			gps.update()
			if not gps.has_fix:
				gps_spd = 'N/A'
			if gps.speed_knots is not None:
				spdmps = gps.speed_knots*0.514444
				gps_spd = '{0:.2f}'.format(spdmps) # Conversion to m/s
			gps_fix = "{}".format(gps.fix_quality)

			# Gather Alpha/Beta values
			for i in range(2):
				a_b[i] = adc.read_adc(i, gain=GAIN)
			a_b[0] = -(a_b[0]*-0.007014328893869 + 44.854179420175704) # Values obtained from calibration
			a_b_string = '{0:.2f}'.format(a_b[0])
			
			# Get pressure data multiple times before next alpha and gps reading
			k = 1
			for g in range(5):
# 				output1 = r1.readline()
# 				output2 = r2.readline()
# 				output3 = r3.readline()
# 				output4 = r4.readline()
# 				output = output1+output2+output3+output4
# 				
# 				p_vals = struct.unpack('>hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh', output)
								
				# Apply calibration value to pitot sensor and divide by integer multiplier, then convert to airspeed
				pitot = 0 #abs(p_vals[51]-CAL[51])/1000 # Apply calibration value to get accurate real time Pitot data from downlink
				pitot = np.sqrt((2*pitot*249.09)/1.13) # 249.09 Pa/inH2O, 1.13 kg/m^3 density in Tucson
				
				# Create string to send over downlink
				downlink = "Reading:\t" + "Alpha: " + a_b_string + "\t" + 'Pitot (m/s): ' + '{0:.2f}'.format(pitot) + "\t" + "GPS Speed (m/s): " + gps_spd + "\t" + "GPS Fix: " + gps_fix
				downlink_simplified = a_b_string + "\t" + '{0:.2f}'.format(pitot) + "\t" + gps_spd + "\t" + gps_fix

				# Create string to print to file
				p_string = "\t".join(map(str,p_vals))	
				filestring = downlink_simplified + '\t' + p_string
				file.write(filestring)
				file.write('\n')
				k = k + 1
			
			# Only send the downlink data every 5 pressure samples
			xbee.write(downlink.encode())
			xbee.write(b'\n')

			if xbee.read(xbee.in_waiting).decode() == 'e':
				#Set GPIO low to indicate end of co
				# llection for NI DAQ
				GPIO.output(24,0)
				print("Data collection complete!")
				xbee.write(str.encode("Data collection complete!"))
				xbee.write(str.encode('\r\n'))
				time.sleep(1)
				break
			else:
				continue
	else:
		pcheck += 1
		if pcheck == 2:
			print("Waiting...")
			xbee.write(str.encode("Waiting:\t"))

			#alph = adc.read_adc(1, gain=GAIN)
			#alph = -(alph*-0.007014328893869 + 44.854179420175704)
			alph = ([0 for q in range(2)])
			for i in range(2):
				alph[i] = adc.read_adc(i, gain=GAIN)
			alph[0] = -(alph[0]*-0.007014328893869 + 44.854179420175704) # Values obtained from calibration
			
			alph_string = 'Alpha: {0:.2f}\t'.format(alph[0])
			xbee.write(str.encode(alph_string))

			gps.update()
			if gps.speed_knots is not None:
				spdmps = gps.speed_knots*0.514444
				gps_spd = 'GPS Speed (m/s): {0:.2f}\t'.format(spdmps) # Conversion to m/s
				xbee.write(str.encode(gps_spd))
			else:
				xbee.write(str.encode('GPS Speed (m/s): N/A\t'))
			xbee.write(str.encode("GPS Fix: {}".format(gps.fix_quality)))
			xbee.write(str.encode('\r\n'))
			pcheck = 0
		time.sleep(0.25)
		continue
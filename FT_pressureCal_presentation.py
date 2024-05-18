# -*- coding: utf-8 -*-
"""
Created on Tue Jun 13 14:55:44 2023

@author: justinjarmer

this file is meant to be run as a subprocess from another file
its purpose is creating a calibration file for the main data accusition loop
"""
import sys
import serial, struct
import numpy as np
import Flight_Test_Functions as FT
import json
import msgpack

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

def manage_json_packet(queue, key, value, send=False):
    queue[key] = value

    if send:
        packet = json.dumps(queue)
        #packet = msgpack.packb(queue)
        print(packet, flush = True)  
        queue.clear()
        
data_json = {}
def main():
    if len(sys.argv) < 3:
        manage_json_packet(data_json, "console", 'Usage Error: python3 FT_pressureCal.py <arg1> <arg2>', True)
        # print('Usage Error: python3 FT_pressureCal.py <arg1> <arg2>')
        sys.stdout.flush()
        return

    try:
        ser1 = serial.Serial('/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0',115200)
        ser2 = serial.Serial('/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0',115200)
        ser3 = serial.Serial('/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0',115200)
        #ser4 = serial.Serial('/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0',115200)
    except Exception as e:
        manage_json_packet(data_json, "console", 'Serial connection to pressure taps could not be opened:', True)
    #     print('Serial connection to pressure taps could not be opened:', str(e))
        sys.stdout.flush()
        return
        
    # Create object for readline wrapper function above
    r1 = ReadLine(ser1)
    r2 = ReadLine(ser2)
    r3 = ReadLine(ser3)
    #r4 = ReadLine(ser4)
    
    location = "/home/pi/Documents/" + sys.argv[1] + "/"
    filename = "CALIBRATION_" + sys.argv[1] + ".txt"
    calfilestring = location + filename
    manage_json_packet(data_json, "console", calfilestring, True)
    # print(calfilestring)
    try:
        calfile = open(calfilestring,'w')    
    except Exception as e:
        ser1.close()
        ser2.close()
        ser3.close()
        #ser4.close()
        manage_json_packet(data_json, "console", 'File could not be opened:' + calfilestring, True)
        # print('File could not be opened:' + calfilestring)#str(e))
        sys.stdout.flush()
        return
    try:
        cal_sample = int(sys.argv[2])
    except:
        cal_sample = 100
    
    cal = [[0 for x in range(52)] for y in range(cal_sample)]
    p = 0
    for k in range(cal_sample):
        output1 = r1.readline()
        output2 = r2.readline()
        output3 = r3.readline()
        #output4 = r4.readline()
        output = output1+output2+output3#+output4
        #print(output)
        manage_json_packet(data_json, "console", len(output), True)
        #print(len(output))
        output = struct.unpack('>hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh', output)
        #output = struct.unpack('>hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh', output)
        cal[k][0:39] = output[0:39]
    
    cal_array = np.array(cal)
    CAL = np.mean(cal_array, axis=0)
    
    calfloat = [0 for x in range(39)]
    calstr = ''
    for i in range(39):
        calfloat[i] = float(CAL[i])
        calstr += '{0:.4f}'.format(CAL[i]) + '\t'
        
    calfile.write(calstr)
    calfile.close()
    
    reply = []
    rep1 = "\n---------------------------------"
    reply.append(rep1)
    rep2 =  "Calibration Complete"
    reply.append(rep2)
    rep3 = calfilestring
    reply.append(rep3)
    reply = "\n".join(reply)
    ser1.close()
    ser2.close()
    ser3.close()
    #ser4.close()
    manage_json_packet(data_json, "console", reply, True)
    manage_json_packet(data_json, 'calibration', calfloat, True)
    sys.stdout.flush()

if __name__ == "__main__":
    main()

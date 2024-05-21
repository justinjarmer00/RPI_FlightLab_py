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

# Wrapper function for pyserial readline function
class ReadLine:
	def __init__(self,s):
		self.buf = bytearray()
		self.s = s

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
        print(packet, flush = True)  
        queue.clear()

def initialize_serial_ports():
    serial_ports = {
        0: '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0',
        1: '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0',
        2: '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0',
        3: '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0'
    }
    connections = {}
    for idx, port in serial_ports.items():
        try:
            ser = serial.Serial(port, 115200)
            connections[idx] = ReadLine(ser)
            manage_json_packet({}, "console", f"Connected to port {idx + 1} ({port})", True)
        except Exception as e:
            manage_json_packet({}, "merror", f"Could not connect to port {idx + 1} ({port}): \n\t{str(e)}", True)
    return connections

data_json = {}

def main():
    if len(sys.argv) < 2:
        manage_json_packet(data_json, "merror", 'Usage Error: python3 FT_pressureCal.py <arg1> <arg2 (optional)>', True)
        sys.stdout.flush()
        return

    port_indices = {
        0: slice(0, 13),   # Indices for serial port 1
        1: slice(13, 26),  # Indices for serial port 2
        2: slice(26, 39),  # Indices for serial port 3
        3: slice(39, 52)   # Indices for serial port 4
    }

    serial_connections = initialize_serial_ports()
    if not serial_connections:
        manage_json_packet(data_json, "merror", "No serial connections available", True)
        sys.exit()

    location = "/home/pi/Documents/" + sys.argv[1] + "/"
    filename = "CALIBRATION_" + sys.argv[1] + ".txt"
    calfilestring = location + filename
    # manage_json_packet(data_json, "console", calfilestring, True)
    
    # Initialize CAL with zeros
    CAL = np.zeros(52)
    try:
        with open(calfilestring, 'w') as calfile:
            iterations = int(sys.argv[2]) if len(sys.argv) > 2 else 100
            cal = [np.zeros(52) for _ in range(iterations)]  # Initialize cal array with zeros for each iteration

            for k in range(iterations):
                for idx in range(4):
                    # manage_json_packet({}, "console", f"Range: {port_indices[idx]}", True)
                    if idx in serial_connections:
                        try:
                            output = serial_connections[idx].readline()
                            if output:
                                unpacked_data = struct.unpack(f'>{13}h', output)
                                # Properly assign unpacked_data to the correct slice in cal array
                                cal[k][port_indices[idx]] = unpacked_data
                        except Exception as e:
                            manage_json_packet({}, "merror", f"Error reading from port {idx+1}: {str(e)}", True)

            # Calculate the mean across all iterations to get the final CAL array
            CAL = np.mean(cal, axis=0)
            # Convert CAL to string with formatted float values
            calstr = '\t'.join(f'{x:.4f}' for x in CAL)
            calfile.write(calstr)            
            reply = [
                "\n---------------------------------",
                "Calibration Complete",
                f"File Location: {calfilestring}",
                f"Number of Serial Connections: {len(serial_connections)}",
                f"Number of Iterations: {iterations}",
            ]
            manage_json_packet({}, "console", "\n".join(reply), True)
            manage_json_packet({}, 'calibration', list(CAL), True)

    except Exception as e:
        for reader in serial_connections.values():
            reader.s.close()
        manage_json_packet({}, "merror", f'File could not be created/opened or other error: {calfilestring}', True)

    sys.stdout.flush()

if __name__ == "__main__":
    main()

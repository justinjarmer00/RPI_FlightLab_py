# -*- coding: utf-8 -*-
"""
Created on Wed Jun 14 14:32:05 2023

@author: justinjarmer
"""

#!/usr/bin/python3
import atexit
import sys, threading, queue
import serial, string, datetime, time, Adafruit_ADS1x15, struct, adafruit_gps
import RPi.GPIO as GPIO
import numpy as np
import json


def cleanup_GPIO():
    GPIO.output(24, 0)
    GPIO.cleanup(24)

atexit.register(cleanup_GPIO)

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
    
    def read_and_clear(self):
        line = self.readline()
        self.s.reset_input_buffer()  # clear the input buffer
        self.buf = bytearray()  # clear the internal buffer
        return line

def manage_json_packet(queue, key, value, send=False):
    queue[key] = value

    if send:
        packet = json.dumps(queue)
        print(packet, flush = True)  # Here, we're printing, but in the actual application, you'd send it over the desired medium.
        queue.clear() 
                
command_queue = queue.Queue()
should_stop = False
main_json = {}
command_json = {}
data_json = {}
def main():
    # print('dataCollection main() started', flush = True)
    manage_json_packet(main_json, "console", "dataCollection main() started", True)
    # Create threads for handling commands and data processing
    command_thread = threading.Thread(target=handle_command)
    data_thread = threading.Thread(target=data_processing)
    
    # Start the threads
    command_thread.start()
    data_thread.start()

    while True:
        # Check if threads are still running
        if not command_thread.is_alive():
            manage_json_packet(main_json, "console", "Command thread has stopped", True)
            #print("Command thread has stopped.", flush = True)
            break
        if not data_thread.is_alive():
            manage_json_packet(main_json, "console", "Data Thread has stopped", True)
            # print("Data thread has stopped.", flush = True)
            should_stop = True
            break
        time.sleep(1)  # Sleep for a second to avoid tight looping

    # Join the threads back to the main thread
    command_thread.join()
    data_thread.join()
    

def handle_command():
    manage_json_packet(command_json, "console", "handle_command() thread started", True)
    # print('handle_command() thread started', flush=True)
    while True:
        if should_stop:
            break
        try:
            manage_json_packet(command_json, "console", "handle_command() waiting...", True)
            # print('handle_command() waiting...', flush=True)
            command = sys.stdin.readline().strip()  # strip to remove trailing newline
            if command:  # only process command if it's not an empty string
                manage_json_packet(command_json, "console", 'received command: ' + command, True)
                # print('received command: ' + command, flush=True)
                command_queue.put(command)
            else:
                manage_json_packet(command_json, "console", "No command received. Likely EOF encountered.", True)
                # print("No command received. Likely EOF encountered.", flush=True)
                break
        except Exception as e:
            manage_json_packet(command_json, "console", f'handle-command() thread broke with error: {str(e)}', True)
            # print(f'handle-command() thread broke with error: {str(e)}', flush=True)
            break
    manage_json_packet(command_json, "console", "handle_command() in dataCollection terminated", True)
    
def data_processing():
    quit_Flag = False
    manage_json_packet(data_json, "console", 'data_processing() started', True)

    #check if all necessary arguments are included
    if len(sys.argv) < 2:
        manage_json_packet(data_json, "console", 'Usage Error: python3 FT_pressureCal.py <arg1> <arg2>', True)
        return
               
    try:
        ser1 = serial.Serial('/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0',115200)
        ser2 = serial.Serial('/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0',115200)
        ser3 = serial.Serial('/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0',115200)
        ser4 = serial.Serial('/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0',115200)
        
        # Create object for readline wrapper function above
        r1 = ReadLine(ser1)
        r2 = ReadLine(ser2)
        r3 = ReadLine(ser3)
        r4 = ReadLine(ser4)

        manage_json_packet(data_json, "console", 'serial initiated', True)
        # print('serial initiated')
    except Exception as e:
        manage_json_packet(data_json, "console", 'Serial connection to pressure taps could not be opened (FT_dataCollection)', True)
        # print('Serial connection to pressure taps could not be opened (FT_dataCollection)' , flush = True)
        quit_Flag = True
    
    # Set values on ADC for alpha beta probe
    adc = Adafruit_ADS1x15.ADS1115()
    GAIN = 2/3
    
    # Create object for readline wrapper function above
    r1 = ReadLine(ser1)
    r2 = ReadLine(ser2)
    r3 = ReadLine(ser3)
    r4 = ReadLine(ser4)
    
    # Set up GPIO to send signal to NI DAQ and trigger the CVA relay
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(24, GPIO.OUT)
    
    # Set up GPS
    GPS_serial = serial.Serial('/dev/ttySOFT0')
    gps = adafruit_gps.GPS(GPS_serial, debug=False) 
    gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
    gps.send_command(b"PMTK220,1000")
    
    #open calibration file
    location = "/home/pi/Documents/" + sys.argv[1] + "/"
    filename = "CALIBRATION_" + sys.argv[1] + ".txt"
    calfilestring = location + filename
    try:
        calfile = open(calfilestring,'r')    
    except Exception as e:
        ser1.close()
        ser2.close()
        ser3.close()
        ser4.close()
        manage_json_packet(data_json, "console", 'Calibration file could not be opened:' + calfilestring, True)
        quit_Flag = True
    CAL_str = calfile.read().split('\t')
    CAL_str = CAL_str[0:52]
    CAL = tuple(map(float,CAL_str))

    p_vals = [0 for i in range(52)]
    pcheck = 0
    start = False
    pitotAvg = 0
    alphAvg = 0
    while True:
        if quit_Flag:
            GPIO.output(24,0)
            break
        
        if not command_queue.empty():
            command = command_queue.get()
            if command == 'start':
                start = True
            elif command == 'end':
                GPIO.output(24,0) # these are here just bc the voltage seems to got to 100% after the flight test is terminated
                start = False
            elif command == 'quit':
                GPIO.output(24,0)
                quit_Flag = True
                manage_json_packet(data_json, "console", "QUIT command recieved", True)
                break
            else:
                manage_json_packet(data_json, "console", 'command not recognized (FT_dataCollection)', True)
        
        if start == True and not quit_Flag:
            k = 0
            manage_json_packet(data_json, "console", "Start command received.", True)
            now = datetime.datetime.now()
            current_time = now.strftime("%H_%M_%S")
            location = "/home/pi/Documents/" + sys.argv[1] + "/"
            filename = "pressure_data_" + current_time + ".txt"
            filestring = location + filename
            file = open(filestring,'w')
    		
            manage_json_packet(data_json, "console", filename + " successfully opened!", True)
            sys.stdout.flush()
            manage_json_packet(data_json, "console", "Reading pressure data...", True)
            sys.stdout.flush()
    		
            file.write("Sensor Numbers \r\n")
            file.write("Alpha\tPitot(m/s)\tGPS(m/s)\tGPS_fix\t1\t2\t3\t4\t5\t6\t7\t8\t9\t10\t11\t12\t13\t14\t15\t16\t17\t18\t19\t20\t21\t22\t23\t24\t25\t26\t27\t28\t29\t30\t31\t32\t33\t34\t35\t36\t37\t38\t39\t40\t41\t42\t43\t44\t45\t46\t47\t48\t49\t50\t51\t52\tTIME\r\n")
            
            startTime = time.perf_counter()
            
            a_b = ([0 for q in range(2)])
            lastTime = startTime
            desired_rate = 1/int(sys.argv[2])
            hz = 0
            while True:
                time_1 = time.perf_counter()
                runTime = time_1 - startTime
                dT = runTime - lastTime
                # listen for commands again
                if not command_queue.empty():
                    command = command_queue.get()
                    if command == 'end':
                        start = False
                        GPIO.output(24,0)
                        break
                    elif command == 'quit':
                        quit_Flag = True
                        GPIO.output(24,0)
                        manage_json_packet(data_json, "console", "QUIT command recieved", True)
                        break
                    else:
                        manage_json_packet(data_json, "console", 'data collection already started, command not recongized (FT_dataCollection)', True)
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
                
                output1 = r1.read_and_clear()
                output2 = r2.read_and_clear()
                output3 = r3.read_and_clear()
                output4 = r4.read_and_clear()
                
                output = output1+output2+output3+output4
				
                p_vals = struct.unpack('>hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh', output)
								
                # Apply calibration value to pitot sensor and divide by integer multiplier, then convert to airspeed
                pitot = abs(p_vals[51]-CAL[51])/1000 # Apply calibration value to get accurate real time Pitot data from downlink
                pitot = np.sqrt((2*pitot*249.09)/1.13) # 249.09 Pa/inH2O, 1.13 kg/m^3 density in Tucson
				
                # Create string to send over downlink
                downlink = "Reading:\t" + "Alpha: " + a_b_string + "\t" + 'Pitot (m/s): ' + '{0:.2f}'.format(pitot) + "\t" + "GPS Speed (m/s): " + gps_spd + "\t" + "GPS Fix: " + gps_fix
                downlink_simplified = a_b_string + "\t" + '{0:.2f}'.format(pitot) + "\t" + gps_spd + "\t" + gps_fix

                # Create string to print to file
                p_string = "\t".join(map(str,p_vals))	
                filestring = downlink_simplified + '\t' + p_string +'\t' + '{0:.2f}'.format(runTime)
                file.write(filestring)
                file.write('\n')
                k = k + 1
    			
                pitotAvg += pitot
                alphAvg += a_b[0]


                try:
                    hz = 1/dT
                except:
                    hz = 0
                # rate at which packets are sent
                if k % round(int(sys.argv[2])/10) == 0:
                    manage_json_packet(data_json, "pitot", pitotAvg/round(int(sys.argv[2])/10))
                    manage_json_packet(data_json, "gps_spd", gps_spd)
                    manage_json_packet(data_json, "gps_fix", gps_fix)
                    manage_json_packet(data_json, "alpha", alphAvg/round(int(sys.argv[2])/10))
                    manage_json_packet(data_json, "p_vals", p_vals)
                    manage_json_packet(data_json, "pi_hz", hz, True)
                    pitotAvg = 0
                    alphAvg = 0
                lastTime = runTime
                time_2 = time.perf_counter() - time_1
                wait_time = desired_rate - time_2
                if wait_time > 0:
                    time.sleep(wait_time)
            manage_json_packet(data_json, "console", "Data collection complete!", True)
            
        else:
            if quit_Flag:
                GPIO.output(24,0)
                break
            
            if len(r1.buf) > 1:
                data_buffer = bytearray()
                while ser1.in_waiting > 0:
                    data = ser1.read(ser1.in_waiting)
                    data_buffer.extend(data)
                
                buffer_size = len(data_buffer)
                manage_json_packet(data_json, "console", f"Length of internal buffer 1: {buffer_size} bytes", True)
                buffer_length = len(r1.buf)
                manage_json_packet(data_json, "console", f"Remaining data in secondary buffer 1: {buffer_length} bytes", True)
                r1.buf = bytearray()  # Clear the buffer by assigning an empty bytearray

            if len(r2.buf) > 1:
                data_buffer = bytearray()
                while ser2.in_waiting > 0:
                    data = ser2.read(ser2.in_waiting)
                    data_buffer.extend(data)
                
                buffer_size = len(data_buffer)
                manage_json_packet(data_json, "console", f"Length of internal buffer 2: {buffer_size} bytes", True)
                buffer_length = len(r2.buf)
                manage_json_packet(data_json, "console", f"Remaining data in secondary buffer 2: {buffer_length} bytes", True)
                r2.buf = bytearray()  # Clear the buffer by assigning an empty bytearray

            if len(r3.buf) > 1:
                data_buffer = bytearray()
                while ser3.in_waiting > 0:
                    data = ser3.read(ser3.in_waiting)
                    data_buffer.extend(data)
                
                buffer_size = len(data_buffer)
                manage_json_packet(data_json, "console", f"Length of internal buffer 3: {buffer_size} bytes", True)
                buffer_length = len(r3.buf)
                manage_json_packet(data_json, "console", f"Remaining data in secondary buffer 3: {buffer_length} bytes", True)
                r3.buf = bytearray()  # Clear the buffer by assigning an empty bytearray

            if len(r4.buf) > 1:
                data_buffer = bytearray()
                while ser4.in_waiting > 0:
                    data = ser4.read(ser4.in_waiting)
                    data_buffer.extend(data)
                
                buffer_size = len(data_buffer)
                manage_json_packet(data_json, "console", f"Length of internal buffer 4: {buffer_size} bytes", True)
                buffer_length = len(r4.buf)
                manage_json_packet(data_json, "console", f"Remaining data in secondary buffer 4: {buffer_length} bytes", True)
                r4.buf = bytearray()  # Clear the buffer by assigning an empty bytearray

            
            pcheck += 1
            if pcheck == 11:
                manage_json_packet(data_json, "console", "Waiting...", True)
                GPIO.output(24,0) #set the voltage to 0, this may be redundant
                alph = ([0 for q in range(2)])
                for i in range(2):
                    alph[i] = adc.read_adc(i, gain=GAIN)
                alph[0] = -(alph[0]*-0.007014328893869 + 44.854179420175704) # Values obtained from calibration
    			
                alph_string = 'Alpha: {0:.2f}\t'.format(alph[0])
                
                output1 = r1.read_and_clear()
                output2 = r2.read_and_clear()
                output3 = r3.read_and_clear()
                output4 = r4.read_and_clear()
                
                output = output1+output2+output3+output4
				
                p_vals = struct.unpack('>hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh', output)	
                # Apply calibration value to pitot sensor and divide by integer multiplier, then convert to airspeed
                pitot = abs(p_vals[51]-CAL[51])/1000 # Apply calibration value to get accurate real time Pitot data from downlink
                pitot = np.sqrt((2*pitot*249.09)/1.13) # 249.09 Pa/inH2O, 1.13 kg/m^3 density in Tucson
                gps.update()
                if gps.speed_knots is not None:
                    spdmps = gps.speed_knots*0.514444
                    gps_spd = '{0:.2f}'.format(spdmps) # Conversion to m/s
                else:
                    gps_spd = 'N/A'
                gps_fix = "{}".format(gps.fix_quality)
                manage_json_packet(data_json, "pitot", pitot)
                manage_json_packet(data_json, "gps_spd", gps_spd)
                manage_json_packet(data_json, "gps_fix", gps_fix)
                manage_json_packet(data_json, "alpha", alph[0])
                manage_json_packet(data_json, "p_vals", p_vals, True)
                pcheck = 0
            time.sleep(0.25)
    #close all refrences
    #GPIO.output(24,0)
    #GPIO.cleanup(24)
    GPS_serial.close()
    ser1.close()
    ser2.close()
    ser3.close()
    ser4.close()
    manage_json_packet(data_json, "console", 'QUIT command recieved. Flight test terminated. Serial conections closed.', True)
    
if __name__ == "__main__":
    main()
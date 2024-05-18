# -*- coding: utf-8 -*-
"""
Created on Wed Jun 14 14:32:05 2023

@author: justinjarmer
"""

#!/usr/bin/python3
import atexit
import sys, threading, queue
import serial, datetime, time, Adafruit_ADS1x15, struct, adafruit_gps
import RPi.GPIO as GPIO
import numpy as np
import json
import adafruit_bno055
import board
import busio
import msgpack

# Handel the issue where signal voltage to sbRIO would go to high on close
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
        #packet = msgpack.packb(queue)
        print(packet, flush = True)  # Here, we're printing, but in the actual application, you'd send it over the desired medium.
        queue.clear() 

command_queue = queue.Queue()
should_stop = False
main_json = {}
command_json = {}
data_json = {}

def main():
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
    while True:
        if should_stop:
            break
        try:
            manage_json_packet(command_json, "console", "handle_command() waiting...", True)
            command = sys.stdin.readline().strip()  # strip to remove trailing newline
            if command:  # only process command if it's not an empty string
                manage_json_packet(command_json, "console", 'received command: ' + command, True)
                command_queue.put(command)
            else:
                manage_json_packet(command_json, "console", "No command received. Likely EOF encountered.", True)
                break
        except Exception as e:
            manage_json_packet(command_json, "console", f'handle-command() thread broke with error: {str(e)}', True)
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
    except Exception as e:
        manage_json_packet(data_json, "console", 'Serial connection to pressure taps could not be opened (FT_dataCollection)', True)
        quit_Flag = True
    
    # Make connection to IMU
    IMUconnected = False
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        IMUsensor = adafruit_bno055.BNO055_I2C(i2c)
        IMUconnected  = True
        IMUaccel = 0 # just in case the first read on the IMU is None for some parameter
        IMUmag = [0, 0, 0]
        IMUgyro = [0, 0, 0]
        IMUeuler = [0, 0, 0]
        IMUquater = [0, 0, 0]
        IMUlinacc = [0, 0, 0]
        IMUgrav = [0, 0, 0]
    except:
        manage_json_packet(data_json, "console", 'connection to IMU could not be made', True)
        IMUaccel = [0, 0, 0]
        IMUmag = [0, 0, 0]
        IMUgyro = [0, 0, 0]
        IMUeuler = [0, 0, 0]
        IMUquater = [0, 0, 0]
        IMUlinacc = [0, 0, 0]
        IMUgrav = [0, 0, 0]
    
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
    GPSconnected = False
    try:
        GPS_serial = serial.Serial('/dev/ttySOFT0')
        gps = adafruit_gps.GPS(GPS_serial, debug=False) 
        gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        gps.send_command(b"PMTK220,1000")
        GPSconnected = True
    except:
        manage_json_packet(data_json, "console", 'connection to GPS could not be made', True)
        gps_fix = 'N/A'
        gps_spd = "N/A"


    
    #open/read calibration file
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

    #loop 1
    while True:
        if quit_Flag:
            GPIO.output(24,0)
            break
        
        if not command_queue.empty():
            command = command_queue.get()
            if command == 'start':
                start = True
            elif command == 'end':
                GPIO.output(24,0) # these are here just bc the voltage seems to go to 100% after the flight test is terminated
                start = False
            elif command == 'quit':
                GPIO.output(24,0)
                quit_Flag = True
                manage_json_packet(data_json, "console", "QUIT command recieved", True)
                break
            else:
                manage_json_packet(data_json, "console", 'command not recognized (FT_dataCollection)', True)
        
        #initialize for loop two
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
            titlept1 = "Alpha\tPitot(m/s)\tGPS(m/s)\tGPS_fix\t1\t2\t3\t4\t5\t6\t7\t8\t9\t10\t11\t12\t13\t14\t15\t16\t17\t18\t19\t20\t21\t22\t23\t24\t25\t26\t27\t28\t29\t30\t31\t32\t33\t34\t35\t36\t37\t38\t39\t40\t41\t42\t43\t44\t45\t46\t47\t48\t49\t50\t51\t52\tTIME"
            titlept2 = "IMUaccelX\tIMUaccelY\tIMUaccelZ\tGyro1\tGyro2\tGyro3\tEuler1\tEuler2\tEuler3\tLinAccelX\tLinAccelY\tLinAccelZ\tGravityX\tGravityY\tGravityZ"
            file.write(titlept1 + "\t" + titlept2 + "\r\n")
            
            startTime = time.perf_counter_ns()
            
            a_b = ([0 for q in range(2)])
            lastTime = startTime
            desired_rate = 1/int(sys.argv[2])
            hz = 0

            # Pre-compile the struct format
            unpacker = struct.Struct('>hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh')

            checkpoint1 = 0
            checkpoint2 = 0
            checkpoint3 = 0
            checkpoint4 = 0
            checkpoint5 = 0
            checkpoint6 = 0
            checkpoint7 = 0
            checkpoint8 = 0
            checkpoint9 = 0
            checkpoint10 = 0
            checkpoint11 = 0
            #loop 2
            while True:
                time_1 = time.perf_counter_ns()
                runTime = time_1 - startTime
                dT = runTime - lastTime
                # listen for commands again
                if not command_queue.empty():
                    command = command_queue.get()
                    if command == 'end':
                        start = False
                        GPIO.output(24,0)
                        # correct checkpoints
                        checkpoint11 -= checkpoint10
                        checkpoint10 -= checkpoint9
                        checkpoint9 -= checkpoint8
                        checkpoint8 -= checkpoint7
                        checkpoint7 -= checkpoint6
                        checkpoint6 -= checkpoint5
                        checkpoint5 -= checkpoint4
                        checkpoint4 -= checkpoint3
                        checkpoint3 -= checkpoint2
                        checkpoint2 -= checkpoint1

                        totalcheck = checkpoint1 + checkpoint2 + checkpoint3 + checkpoint4 + checkpoint5 + checkpoint6 + checkpoint7 + checkpoint8 + checkpoint9 + checkpoint10 + checkpoint11
                        totalstring = ('\naverage time in ns:\n1:\t' + str(checkpoint1/k) + 
                                        '\n2:\t' + str(checkpoint2/k) +
                                        '\n3:\t' + str(checkpoint3/k) +
                                        '\n4:\t' + str(checkpoint4/k) +
                                        '\n5:\t' + str(checkpoint5/k) +
                                        '\n6:\t' + str(checkpoint6/k) +
                                        '\n7:\t' + str(checkpoint7/k) +
                                        '\n8:\t' + str(checkpoint8/k) +
                                        '\n9:\t' + str(checkpoint9/k) +
                                        '\n10:\t' + str(checkpoint10/k) + 
                                        '\n11:\t' + str(checkpoint11/k) + 
                                        '\ntotal:\t' + str(totalcheck/k)) # k is the number of loops
                        proportionstring = ('\nproportional time:\n1:\t' + str(checkpoint1/totalcheck) + 
                                        '\n2:\t' + str(checkpoint2/totalcheck) +
                                        '\n3:\t' + str(checkpoint3/totalcheck) +
                                        '\n4:\t' + str(checkpoint4/totalcheck) +
                                        '\n5:\t' + str(checkpoint5/totalcheck) +
                                        '\n6:\t' + str(checkpoint6/totalcheck) +
                                        '\n7:\t' + str(checkpoint7/totalcheck) +
                                        '\n8:\t' + str(checkpoint8/totalcheck) +
                                        '\n9:\t' + str(checkpoint9/totalcheck) +
                                        '\n10:\t' + str(checkpoint10/totalcheck) +
                                        '\n11:\t' + str(checkpoint11/totalcheck))

                        manage_json_packet(data_json, "console", totalstring, True)
                        manage_json_packet(data_json, "console", proportionstring, True)
                        break
                    elif command == 'quit':
                        quit_Flag = True
                        # GPIO.output(24,0)
                        manage_json_packet(data_json, "console", "QUIT command recieved", True)
                        break
                    else:
                        manage_json_packet(data_json, "console", 'data collection already started, command not recongized (FT_dataCollection)', True)
                # Set GPIO high to indicate start of collection for NI DAQ
                checkpoint1 += time.perf_counter_ns() - time_1
                GPIO.output(24,1)
    
                # Get GPS update
                if GPSconnected:
                    gps.update()
                    if not gps.has_fix:
                        gps_spd = 'N/A'
                    if gps.speed_knots is not None:
                        spdmps = gps.speed_knots*0.514444
                        gps_spd = '{0:.2f}'.format(spdmps) # Conversion to m/s
                    gps_fix = "{}".format(gps.fix_quality)

                checkpoint2 += time.perf_counter_ns() - time_1
                # Gather Alpha/Beta values
                if k % 2 == 0:
                    for i in range(1): # use in range(2) to get beta as well, this adc.read_adc however is a time consuming command
                        a_b[i] = adc.read_adc(i, gain=GAIN)
                    a_b[0] = -(a_b[0]*-0.007014328893869 + 44.854179420175704) # Values obtained from calibration
                    a_b_string = '{0:.2f}'.format(a_b[0])
                
                checkpoint3 += time.perf_counter_ns() - time_1
                # read data from IMU
                if IMUconnected: 
                    #manage_json_packet(data_json, "console", "Reading IMU data...", True)
                    newIMUaccel = IMUsensor.acceleration
                    newIMUmag = IMUsensor.magnetic
                    newIMUgyro = IMUsensor.gyro
                    newIMUeuler = IMUsensor.euler
                    newIMUquater = IMUsensor.quaternion
                    newIMUlinacc = IMUsensor.linear_acceleration
                    newIMUgrav = IMUsensor.gravity
                    IMUaccel = newIMUaccel if all(value is not None for value in newIMUaccel) else IMUaccel
                    IMUmag = newIMUmag if all(value is not None for value in newIMUmag) else IMUmag
                    IMUgyro = newIMUgyro if all(value is not None for value in newIMUgyro) else IMUgyro
                    IMUeuler = newIMUeuler if all(value is not None for value in newIMUeuler) else IMUeuler
                    IMUquater = newIMUquater if all(value is not None for value in newIMUquater) else IMUquater
                    IMUlinacc = newIMUlinacc if all(value is not None for value in newIMUlinacc) else IMUlinacc
                    IMUgrav = newIMUgrav if all(value is not None for value in newIMUgrav) else IMUgrav
                    #manage_json_packet(data_json, "console", '{}'.format(IMUaccel), True)
                    #manage_json_packet(data_json, "console", '{}'.format(IMUgyro), True)
                    #manage_json_packet(data_json, "console", '{}'.format(IMUeuler), True)
                    #manage_json_packet(data_json, "console", '{}'.format(IMUlinacc), True)
                    #manage_json_packet(data_json, "console", '{}'.format(IMUgrav), True)
                    #manage_json_packet(data_json, "console", '{}'.format(IMUmag), True)

                checkpoint4 += time.perf_counter_ns() - time_1
                output1 = r1.read_and_clear()
                output2 = r2.read_and_clear()
                output3 = r3.read_and_clear()
                output4 = r4.read_and_clear()
                
                checkpoint5 += time.perf_counter_ns() - time_1

                output = output1+output2+output3+output4
				
                # Use the pre-compiled unpacker
                p_vals = unpacker.unpack(output)
                # p_vals = struct.unpack('>hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh', output)

                checkpoint6 += time.perf_counter_ns() - time_1

                # Apply calibration value to pitot sensor and divide by integer multiplier, then convert to airspeed
                pitot = abs(p_vals[51]-CAL[51])/1000 # Apply calibration value to get accurate real time Pitot data from downlink
                pitot = np.sqrt((2*pitot*249.09)/1.13) # 249.09 Pa/inH2O, 1.13 kg/m^3 density in Tucson
				
                checkpoint7 += time.perf_counter_ns() - time_1

                # Create string to send over downlink
                # downlink = "Reading:\t" + "Alpha: " + a_b_string + "\t" + 'Pitot (m/s): ' + '{0:.2f}'.format(pitot) + "\t" + "GPS Speed (m/s): " + gps_spd + "\t" + "GPS Fix: " + gps_fix
                downlink_pt1 = a_b_string + "\t" + '{0:.2f}'.format(pitot) + "\t" + gps_spd + "\t" + gps_fix
                downlink_pt2 = '{0:.3f}'.format(IMUaccel[0]) + "\t" + '{0:.3f}'.format(IMUaccel[1])  + "\t" + '{0:.3f}'.format(IMUaccel[2]) + "\t"
                downlink_pt3 = '{0:.6f}'.format(IMUgyro[0]) + "\t" + '{0:.6f}'.format(IMUgyro[1])  + "\t" + '{0:.6f}'.format(IMUgyro[2]) + "\t"
                downlink_pt4 = '{0:.3f}'.format(IMUeuler[0]) + "\t" + '{0:.3f}'.format(IMUeuler[1])  + "\t" + '{0:.3f}'.format(IMUeuler[2]) + "\t"
                downlink_pt5 = '{0:.3f}'.format(IMUlinacc[0]) + "\t" + '{0:.3f}'.format(IMUlinacc[1])  + "\t" + '{0:.3f}'.format(IMUlinacc[2]) + "\t"
                downlink_pt6 = '{0:.3f}'.format(IMUgrav[0]) + "\t" + '{0:.3f}'.format(IMUgrav[1])  + "\t" + '{0:.3f}'.format(IMUgrav[2])
                #manage_json_packet(data_json, "console", "Created downlinks", True)
                checkpoint8 += time.perf_counter_ns() - time_1

                # Create string to print to file
                p_string = "\t".join(map(str,p_vals))	
                filestring = downlink_pt1 + '\t' + p_string +'\t' + '{0:.3f}'.format(runTime/1000000000) + downlink_pt2  + downlink_pt3 + downlink_pt4 + downlink_pt5 + downlink_pt6
                file.write(filestring)
                file.write('\n')
                k = k + 1
    			
                pitotAvg += pitot
                alphAvg += a_b[0]

                checkpoint9 += time.perf_counter_ns() - time_1

                try:
                    hz = 1000000000/dT #billion ns in a sec
                except:
                    hz = 0
                # rate at which packets are sent
                sendHz = 5
                if k % round(int(sys.argv[2])/sendHz) == 0:
                    #manage_json_packet(data_json, "console", "Begining data send", True)
                    manage_json_packet(data_json, "p", int(10*pitotAvg/round(int(sys.argv[2])/sendHz)))
                    manage_json_packet(data_json, "gs", gps_spd)
                    manage_json_packet(data_json, "gf", gps_fix)
                    manage_json_packet(data_json, "a", int(10*alphAvg/round(int(sys.argv[2])/sendHz)))
                    manage_json_packet(data_json, "pv", [int(x / 10) for x in p_vals])
                    
                    #manage_json_packet(data_json, "IMUaccel", IMUaccel)
                    #manage_json_packet(data_json, "IMUmag", IMUmag)
                    #manage_json_packet(data_json, "IMUgyro", IMUgyro)
                    #manage_json_packet(data_json, "IMUeuler", IMUeuler)
                    #manage_json_packet(data_json, "IMUlinacc", IMUlinacc)
                    #manage_json_packet(data_json, "IMUgrav", IMUgrav)

                    manage_json_packet(data_json, "h", int(hz), True)
                    pitotAvg = 0
                    alphAvg = 0
                    #manage_json_packet(data_json, "console", "Finished Data Send", True)
                checkpoint10 += time.perf_counter_ns() - time_1
                lastTime = runTime
                time_f = time.perf_counter_ns() - time_1
                # wait_time = desired_rate - time_f/(1000000000) #get from nanosec to sec
                # if wait_time > 0:
                #     time.sleep(wait_time)
                target_time = time_1 + 1000000000*desired_rate  # Convert desired_rate to ns
                while time.perf_counter_ns() < target_time:
                    pass  # Busy-wait until the target time is reached

                checkpoint11 += time.perf_counter_ns() - time_1
            manage_json_packet(data_json, "console", "Data collection complete!", True)
            
        else:
            if quit_Flag:
                GPIO.output(24,0)
                break
            
            # the folowing will only trigger if there is left over data in the buffers after the data collection process is finished. THIS SHOULD NOT HAPPEN! the code was designed so that nothing would be left. but just in case i am going to check for it
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
                if IMUconnected: 
                    manage_json_packet(data_json, "console", 'IMU calibrated? {0}... Status:{1}'.format(IMUsensor.calibrated,IMUsensor.calibration_status), True)
                    #manage_json_packet(data_json, "console", 'Calibration Status: {}'.format(IMUsensor.calibration_status), True)

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
                # Get GPS update
                if GPSconnected:
                    gps.update()
                    if gps.speed_knots is not None:
                        spdmps = gps.speed_knots*0.514444
                        gps_spd = '{0:.2f}'.format(spdmps) # Conversion to m/s
                    else:
                        gps_spd = 'N/A'
                    gps_fix = "{}".format(gps.fix_quality)
                manage_json_packet(data_json, "p", int(pitot*10))
                manage_json_packet(data_json, "gs", gps_spd)
                manage_json_packet(data_json, "gf", gps_fix)
                manage_json_packet(data_json, "a", int(alph[0]*10))
                manage_json_packet(data_json, "pv", [int(x / 10) for x in p_vals])
                manage_json_packet(data_json, "h", 0, True)
                pcheck = 0
            time.sleep(0.07)
    #close all refrences
    GPS_serial.close()
    ser1.close()
    ser2.close()
    ser3.close()
    ser4.close()
    manage_json_packet(data_json, "console", 'QUIT command recieved. Flight test terminated. Serial conections closed.', True)
    
if __name__ == "__main__":
    main()
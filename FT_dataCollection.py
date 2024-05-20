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

# Handle the issue where signal voltage to sbRIO would go to high on close
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

def handle_error(message, exception=None):
    if exception:
        manage_json_packet({}, "merror", f"{message}: {str(exception)}", True)
    else:
        manage_json_packet({}, "merror", message, True)

def initialize_serial_ports():
    serial_ports = {
        0: '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0',
        1: '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0',
        2: '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0',
        3: '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0'
    }
    serial_connections = {}
    read_lines = {}

    for idx, port in serial_ports.items():
        try:
            ser = serial.Serial(port, 115200)
            serial_connections[idx] = ser
            read_lines[idx] = ReadLine(ser)
            manage_json_packet({}, "console", f"Connected to port {idx+1} ({port})", True)
        except Exception as e:
            handle_error(f"Could not connect to port {idx+1} ({port})", e)

    return serial_connections, read_lines

def initialize_imu():
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        IMUsensor = adafruit_bno055.BNO055_I2C(i2c)
        return IMUsensor, True
    except Exception as e:
        handle_error('Connection to IMU could not be made', e)
        return None, False

def initialize_gps():
    try:
        GPS_serial = serial.Serial('/dev/ttySOFT0')
        gps = adafruit_gps.GPS(GPS_serial, debug=False)
        gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        gps.send_command(b"PMTK220,1000")
        return GPS_serial, gps, True
    except Exception as e:
        handle_error('Connection to GPS could not be made', e)
        return None, False
    
def initialize_adc():
    try:
        adc = Adafruit_ADS1x15.ADS1115()
        GAIN = 2/3
        manage_json_packet({}, "console", "ADC initialized successfully", True)
        return adc, GAIN, True
    except Exception as e:
        handle_error('Connection to ADC could not be made', e)
        return None, None, False

# def create_unpacker(num_connections):
#     # Assuming each connection reads 13 short values
#     num_values = num_connections * 13
#     return struct.Struct(f'>{num_values}h')

main_json = {}
# command_json = {}
data_json = {}

def main():
    manage_json_packet({}, "console", "dataCollection main() started", True)
    # Create threads for handling commands and data processing
    command_thread = threading.Thread(target=handle_command)
    data_thread = threading.Thread(target=data_processing)
    command_thread.start()
    data_thread.start()

    while True:
        if not command_thread.is_alive() or not data_thread.is_alive():
            handle_error("One of the threads has stopped for Data Collection.")
            should_stop = True
            break
        time.sleep(1) # Sleep for a second to avoid tight looping

     # Join the threads back to the main thread
    command_thread.join()
    data_thread.join()

def handle_command():
    manage_json_packet({}, "console", "handle_command() thread started", True)
    while not should_stop:
        try:
            manage_json_packet({}, "console", "handle_command() waiting...", True)
            command = sys.stdin.readline().strip() # strip to remove trailing newline
            if command: # only process command if it's not an empty string
                manage_json_packet({}, "console", f'Received command: {command}', True)
                command_queue.put(command)
        except Exception as e:
            handle_error('handle_command() thread encountered an error', e)
            break
    manage_json_packet({}, "console", "handle_command() thread terminated", True)
    
def data_processing():
    quit_Flag = False
    manage_json_packet({}, "console", 'data_processing() started', True)

    #check if all necessary arguments are included
    if len(sys.argv) < 2:
        handle_error('Usage Error: python3 FT_dataCollection.py <arg1> <arg2>')
        return

    # Attempt to initialize up to 4 serial connections
    serial_connections, read_lines = initialize_serial_ports()
    if not serial_connections:
        handle_error("No serial connections available")

    # Make connection to IMU
    IMUsensor, IMUconnected = initialize_imu()
    if not IMUconnected:
        handle_error("No IMU connection available")
    
    # Set up GPS
    GPS_serial, gps, GPSconnected = initialize_gps()
    if not GPSconnected:
        handle_error("No GPS connection available")
    
    # Set values on ADC for alpha beta probe
    adc, GAIN, ADCconnected = initialize_adc()
    if not ADCconnected:
        handle_error("No ADC connection available")

    # Set up GPIO to send signal to NI DAQ and trigger the CVA relay
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(24, GPIO.OUT)

    # Define the default CAL array size
    CAL_SIZE = 52

    # Initialize CAL array with zeros
    CAL = tuple([0.0] * CAL_SIZE)

    # Construct file path
    location = "/home/pi/Documents/" + sys.argv[1] + "/"
    filename = "CALIBRATION_" + sys.argv[1] + ".txt"
    calfilestring = location + filename

    try:
        # Attempt to open the calibration file
        with open(calfilestring, 'r') as calfile:
            CAL_str = calfile.read().split('\t')
            if len(CAL_str) >= CAL_SIZE:
                # Only update CAL if the file provides enough data
                CAL = tuple(map(float, CAL_str[:CAL_SIZE]))
            else:
                # Handle case where not enough calibration data is available
                handle_error(f'Insufficient calibration data in file: {calfilestring}', None)
    except Exception as e:
        # Handle file open/read errors
        handle_error(f'Calibration file could not be opened or read: {calfilestring}', e)

    # Continue with program execution

    start = False

    while not quit_flag:
        if not command_queue.empty():
            command = command_queue.get()
            if command == 'start':
                start = True
            elif command == 'end':
                GPIO.output(24, 0)
                start = False
            elif command == 'quit':
                GPIO.output(24, 0)
                quit_flag = True
                manage_json_packet({}, "console", "QUIT command received", True)
                break
            else:
                manage_json_packet({}, "console", 'Command not recognized', True)

        if start and not quit_flag:
            run_data_collection(CAL, serial_connections, read_lines, gps, GPSconnected, adc, GAIN, ADCconnected, IMUsensor, IMUconnected)

        time.sleep(0.07)

    close_connections(serial_connections, GPS_serial)

def run_data_collection(CAL, serial_connections, read_lines, gps, GPSconnected, adc, GAIN, ADCconnected, IMUsensor, IMUconnected):
    k = 0
    now = datetime.datetime.now()
    current_time = now.strftime("%H_%M_%S")
    location = "/home/pi/Documents/" + sys.argv[1] + "/"
    filename = "pressure_data_" + current_time + ".txt"
    filestring = location + filename
    try:
        file = open(filestring, 'w')
    except Exception as e:
        handle_error(f'File could not be created/opened: {filestring}', e)
        return

    file.write("Sensor Numbers \r\n")
    titlept1 = "Alpha\tPitot(m/s)\tGPS(m/s)\tGPS_fix\t1\t2\t3\t4\t5\t6\t7\t8\t9\t10\t11\t12\t13\t14\t15\t16\t17\t18\t19\t20\t21\t22\t23\t24\t25\t26\t27\t28\t29\t30\t31\t32\t33\t34\t35\t36\t37\t38\t39\t40\t41\t42\t43\t44\t45\t46\t47\t48\t49\t50\t51\t52\tTIME"
    titlept2 = "IMUaccelX\tIMUaccelY\tIMUaccelZ\tGyro1\tGyro2\tGyro3\tEuler1\tEuler2\tEuler3\tLinAccelX\tLinAccelY\tLinAccelZ\tGravityX\tGravityY\tGravityZ"
    file.write(titlept1 + "\t" + titlept2 + "\r\n")

    start_time = time.perf_counter_ns()
    a_b = [0 for _ in range(2)]
    last_time = start_time
    desired_rate = 1 / int(sys.argv[2])
    hz = 0
    
    # Create dynamic unpacker
    # unpacker = create_unpacker(len(serial_connections))

    # Initialize IMU variables with default numeric values
    IMUaccel = [0, 0, 0]
    IMUmag = [0, 0, 0]
    IMUgyro = [0, 0, 0]
    IMUeuler = [0, 0, 0]
    IMUquater = [0, 0, 0, 0]
    IMUlinacc = [0, 0, 0]
    IMUgrav = [0, 0, 0]

    p_vals = CAL.copy()

    port_indices = {
        0: slice(0, 13),   # Indices for serial port 1
        1: slice(13, 26),  # Indices for serial port 2
        2: slice(26, 39),  # Indices for serial port 3
        3: slice(39, 52)   # Indices for serial port 4
    }

    while True:
        try:
            if not command_queue.empty():
                command = command_queue.get()
                if command == 'end':
                    break
                elif command == 'quit':
                    quit_flag = True
                    break

            time_1 = time.perf_counter_ns()
            run_time = time_1 - start_time
            dT = run_time - last_time

            GPIO.output(24, 1)

            if GPSconnected:
                try:
                    gps.update()
                    gps_spd = '{0:.2f}'.format(gps.speed_knots * 0.514444) if gps.speed_knots else 'None'
                    gps_fix = "{}".format(gps.fix_quality) if gps.has_fix else 'None'
                except Exception as e:
                    GPSconnected = False
                    gps_spd = 'N/A'
                    gps_fix = 'N/A'
                    handle_error('GPS disconnected or failed to update', e)

            if k % 2 == 0 and ADCconnected:
                try:
                    for i in range(1):
                        a_b[i] = adc.read_adc(i, gain=GAIN)
                    a_b[0] = -(a_b[0] * -0.007014328893869 + 44.854179420175704)
                    a_b_string = '{0:.2f}'.format(a_b[0])
                except Exception as e:
                    ADCconnected = False
                    a_b_string = '999.99'
                    handle_error('ADC disconnected or failed to read', e)

            if IMUconnected:
                try:
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
                except Exception as e:
                    IMUconnected = False
                    handle_error('IMU disconnected or failed to read', e)

            for idx in range(4):  # Assuming 4 ports as defined
                if idx in serial_connections:
                    try:
                        unpacker = struct.Struct(f'>{13}h')
                        data = read_lines[idx].read_and_clear()
                        unpacked_data = unpacker.unpack(data)
                        p_vals[port_indices[idx]] = unpacked_data[:len(port_indices[idx])]
                    except Exception as e:
                        handle_error(f'Serial connection {idx+1} read failed', e)
                        serial_connections[idx].close()
                        del serial_connections[idx]
                        del read_lines[idx]
                        p_vals[port_indices[idx]] = CAL[port_indices[idx]]
                else:
                    # If the connection is missing or has been removed, fill with CAL values
                    p_vals[port_indices[idx]] = CAL[port_indices[idx]]

            # Read from serial connections
            # valid_data = []
            # for i, (ser, reader) in enumerate(zip(serial_connections, read_lines)):
            #     try:
            #         data = reader.read_and_clear()
            #         valid_data.append(data)
            #     except Exception as e:
            #         handle_error(f'Serial connection {i+1} read failed', e)
            #         ser.close()
            #         serial_connections.pop(i)
            #         read_lines.pop(i)
            #         unpacker = create_unpacker(len(serial_connections))

            # if valid_data:
            #     output = b"".join(valid_data)
            #     try:
            #         p_vals = unpacker.unpack(output)
            #     except Exception as e:
            #         handle_error('Unpacking serial data failed', e)
            #         continue  # Skip processing if unpacking fails

            # output = b"".join([r.read_and_clear() for r in read_lines])
            # p_vals = unpacker.unpack(output)

            pitot = abs(p_vals[51] - CAL[51]) / 1000
            pitot = np.sqrt((2 * pitot * 249.09) / 1.13)

            downlink_pt1 = a_b_string + "\t" + '{0:.2f}'.format(pitot) + "\t" + gps_spd + "\t" + gps_fix
            downlink_pt2 = '{0:.3f}'.format(IMUaccel[0]) + "\t" + '{0:.3f}'.format(IMUaccel[1]) + "\t" + '{0:.3f}'.format(IMUaccel[2]) + "\t"
            downlink_pt3 = '{0:.6f}'.format(IMUgyro[0]) + "\t" + '{0:.6f}'.format(IMUgyro[1]) + "\t" + '{0:.6f}'.format(IMUgyro[2]) + "\t"
            downlink_pt4 = '{0:.3f}'.format(IMUeuler[0]) + "\t" + '{0:.3f}'.format(IMUeuler[1]) + "\t" + '{0:.3f}'.format(IMUeuler[2]) + "\t"
            downlink_pt5 = '{0:.3f}'.format(IMUlinacc[0]) + "\t" + '{0:.3f}'.format(IMUlinacc[1]) + "\t" + '{0:.3f}'.format(IMUlinacc[2]) + "\t"
            downlink_pt6 = '{0:.3f}'.format(IMUgrav[0]) + "\t" + '{0:.3f}'.format(IMUgrav[1]) + "\t" + '{0:.3f}'.format(IMUgrav[2])

            p_string = "\t".join(map(str, p_vals))
            filestring = downlink_pt1 + '\t' + p_string + '\t' + '{0:.3f}'.format(run_time / 1000000000) + downlink_pt2 + downlink_pt3 + downlink_pt4 + downlink_pt5 + downlink_pt6
            file.write(filestring + '\n')

            pitot_avg += pitot
            alpha_avg += a_b[0]

            try:
                hz = 1000000000 / dT
            except:
                hz = 0

            sendHz = 5
            if k % round(int(sys.argv[2]) / sendHz) == 0:
                manage_json_packet(data_json, "p", int(10 * pitot_avg / round(int(sys.argv[2]) / sendHz)))
                manage_json_packet(data_json, "gs", gps_spd)
                manage_json_packet(data_json, "gf", gps_fix)
                manage_json_packet(data_json, "a", int(10 * alpha_avg / round(int(sys.argv[2]) / sendHz)))
                manage_json_packet(data_json, "pv", [int(x / 10) for x in p_vals])
                manage_json_packet(data_json, "h", int(hz), True)
                pitot_avg = 0
                alpha_avg = 0

            last_time = run_time
            target_time = time_1 + 1000000000 * desired_rate
            while time.perf_counter_ns() < target_time:
                pass

        except Exception as e:
            handle_error('Error during data collection loop', e)

    file.close()

def close_connections(serial_connections, GPS_serial):
    # Iterate over the serial connection objects stored in the values of the dictionary
    for ser in serial_connections.values():
        ser.close()
    if GPS_serial:
        GPS_serial.close()
    manage_json_packet({}, "console", 'Serial connections closed.', True)

command_queue = queue.Queue()
should_stop = False

if __name__ == "__main__":
    main()



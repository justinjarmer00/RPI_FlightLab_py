# -*- coding: utf-8 -*-
"""
Created on Tue Jun 13 11:35:49 2023

@author: justinjarmer

The goal of this file is to act as the conection between the computer and the rasberry pi. 
The idea is that this file will listen for messages from the computer and call functions (or other codes from there).

"""
# import os, serial, string, datetime, time, Adafruit_ADS1x15, struct, adafruit_gps
# import RPi.GPIO as GPIO
# import numpy as np

import time
import serial
import os
import subprocess
import threading
import queue
import sys
import json
import msgpack
sys.path.append('/home/pi/Flight_Test')
import Flight_Test_Functions as FT

def manage_json_packet(queue, xbee, key, value, send=False):
    queue[key] = value

    if send:
        #packet = json.dumps(queue)
        packet = msgpack.packb(queue)
        FT.sendXbee(xbee, packet)
        # print(packet, flush = True)  # Here, we're printing, but in the actual application, you'd send it over the desired medium.
        queue.clear()

print('starting...')
time.sleep(2)

xbee = serial.Serial(
    port ='/dev/ttyS0',  #/dev/ttyS0',
    baudrate = 9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

# Get the absolute path of the directory of the current script
current_directory = os.path.dirname(os.path.realpath(__file__))
selectedDir = 'none'

# A queue to hold the commands received from the XBee
command_queue = queue.Queue()

def is_number(value):
    try:
        float(value)
        return True
    except:
        return False 

message_json = {}

def listen_for_commands():
    print('listen_for_commands() started')
    while True:
        raw_message = FT.readXbee(xbee)
        message = raw_message.split()
        if message and message[0] != 'none':
            command_queue.put(message)
        if message[0] == 'exit':
            return
        time.sleep(0.05)

def handle_commands():
    print('handle_commands() started')
    current_directory = os.path.dirname(os.path.realpath(__file__))
    selectedDir = 'none'
    flight_process = None
    while True:
        # Wait until there's a command in the queue and remove it
        command = command_queue.get()

        if command[0] == 'help' or command[0] == 'hello':
            
            reply = ("Hello!\n"
            "Bellow are the commands for this program\n"
            "--> mkdir dir_name\n"
            "\tThis will make a directory named \"dir_name\" in pi/Documents\n"
            "--> select_dir dir_name\n"
            "\tThis will select a directory \"dir_name\" in pi/Documents for future files\n"
            "\tIf the dir_name is excluded the current selected dir will be returned\n"
            "--> calc_CG FW BLW BRW\n"
            "\tThis will calculate the CG and SM (6/2023).\n"
            "\tFW, BLW, BRW, are recorded weights at each wheel\n"
            "--> calibrate num_samples\n"
            "\tThis will create a calibration file in the selected directory\n"
            "\tIf num_samples can not be read, 100 will be used\n"
            "--> run_flight_test\n"
            "\tThis will begin the collection waiting sequence\n"
            "--> start\n"
            "\tThis will begin the collection process if a flight test has already been started\n"
            "--> end\n"
            "\tThis will end the collection process and return it to its waiting loop\n"
            "--> quit\n"
            "\tThis will terminate the flight test waiting and collection processes\n"
            "\tRun this if the flight test is not listening to other commands\n"
            "\tAlso run this if the flight test closed with an error\n"
            "--> exit\n"
            "\tUse to close the program, it will run on next startup")
            
            # FT.sendXbee(xbee, reply)
            manage_json_packet(message_json, xbee, 'console', reply, True)
        elif command[0] == 'mkdir':
            path = '/home/pi/Documents/'
            reply = ''
            try:
                FT.mkdir(path, command[1])
                reply = 'made directory: ' + path + command[1]
                selectedDir = command[1]                    
            except:
                reply = 'no directory name given'
            # FT.sendXbee(xbee, reply) 
            manage_json_packet(message_json, xbee, 'console', reply, True)
        elif command[0] == 'ping':
            manage_json_packet(message_json, xbee, 'q', 'a', True)
        elif command[0] == 'calc_CG':
            reply = ''
            try:
                reply = FT.calculateCG(float(command[1]),float(command[2]),float(command[3]))
            except:
                reply = '\nerror with calc_CG call\tmake sure inputs are correct'
            # FT.sendXbee(xbee, reply)
            manage_json_packet(message_json, xbee, 'console', reply, True)
        elif command[0] == 'calibrate':
            if len(command) >= 2 and selectedDir != 'none':
                # Construct the absolute path of the script to be executed
                script_to_execute = os.path.join(current_directory, "FT_pressureCal_presentation.py")
                process = subprocess.Popen(["python3", "-u", script_to_execute, selectedDir, command[1]], stdout=subprocess.PIPE, text=True)
                # Start a new thread to wait for the subprocess to finish and handle its output
                # threading.Thread(target=handle_subprocess_output, args=(process,)).start()
                threading.Thread(target=handle_stream_subprocess_output, args=(process,)).start()
                reply = 'calibration command recieved'
            else:
                reply = 'error beginning subprocess: directory not selected?'
                # FT.sendXbee(xbee, reply)
            manage_json_packet(message_json, xbee, 'console', reply, True)
        elif command[0] == 'error':
            reply = ''
            try:
                reply = command[1] + ' ' + command[0]
            except:
                reply = 'error with error'
            # FT.sendXbee(xbee, reply)
            manage_json_packet(message_json, xbee, 'console', reply, True)
        elif command[0] == 'select_dir':
            reply = ''
            try: 
                reply = 'selected directory: ' + os.path.join(current_directory, command[1])
                selectedDir = command[1]
            except:
                reply = 'error selecting new directory | current directory: ' + selectedDir
            # FT.sendXbee(xbee, reply)
            manage_json_packet(message_json, xbee, 'console', reply, True)
        elif command[0] == 'run_flight_test':
            if len(command) >= 1 and selectedDir != 'none':
                rateHz = '20' # default sample rate
                try:
                    if  is_number(command[1]):
                        rateHz = command[1]
                except:
                    reply = 'default Hz will be used'
                script_to_execute = os.path.join(current_directory, "FT_dataCollection_presentation.py")
                flight_process = subprocess.Popen(["python3", script_to_execute, selectedDir, rateHz], 
                           stdin=subprocess.PIPE, 
                           stdout=subprocess.PIPE, 
                           stderr=subprocess.PIPE,
                           text=True)
                
                # processes['flight_test'] = process
                # print(processes)
                #flight_process.stdin.write('command[0]')
                #flight_process.stdin.flush()
                threading.Thread(target=handle_stream_subprocess_output, args=(flight_process,)).start()
                threading.Thread(target=handle_stream_subprocess_error, args=(flight_process,)).start()
            else:
                reply = 'error beginning subprocess: directory not selected?'
                # FT.sendXbee(xbee, reply)
            manage_json_packet(message_json, xbee, 'console', reply, True)
        elif command[0] == 'start':
            # print(processes)
            # process = flight_process
            reply = ''
            if flight_process is not None:
                flight_process.stdin.write((command[0] + '\n')) # We add a newline to ensure readline() can read it properly
                flight_process.stdin.flush()
                print('recieved start command')
                reply = 'recieved start command'
            else:
                reply = 'the flight_test process has not been started'
                # FT.sendXbee(xbee, reply)
            manage_json_packet(message_json, xbee, 'console', reply, True)
        elif command[0] == 'end':
            # process = processes.get('flight_test')
            reply = ''
            if flight_process is not None:
                flight_process.stdin.write(command[0] + '\n')
                flight_process.stdin.flush()
                reply = 'end command recieved'
            else:
                reply = 'the flight_test process has not been started'
                # FT.sendXbee(xbee, reply)
            manage_json_packet(message_json, xbee, 'console', reply, True)
        elif command[0] == 'quit':
            # process = processes.get('flight_test')
            reply = ''
            if flight_process is not None:
                flight_process.stdin.write(command[0] + '\n')
                flight_process.stdin.flush()
                flight_process = None
                reply = 'quit command recieved'
            else:
                reply = 'the flight_test process has not been started'
                # FT.sendXbee(xbee, reply)
            manage_json_packet(message_json, xbee, 'console', reply, True)
        elif command[0] == 'exit':
            reply = 'closing program...'
            # FT.sendXbee(xbee, reply)
            manage_json_packet(message_json, xbee, 'console', reply, True)
            xbee.close()
            break
        else:
            reply = '\nnot a recongized command: type "help" for list of accepted commands'
            # FT.sendXbee(xbee,reply)
            manage_json_packet(message_json, xbee, 'console', reply, True)
            print(command)
            
def handle_subprocess_output(process):
    # Wait for the subprocess to finish and get its output
    output, _ = process.communicate()
    # Send the output to the XBee
    # Parse the JSON string back to a Python object
    data_object = json.loads(output)

    # Convert the object to a MessagePack byte array
    packed_data = msgpack.packb(data_object)

    # Send the MessagePack data
    FT.sendXbee(xbee, packed_data)

def handle_stream_subprocess_output(process):
    # Loop until the subprocess closes its output
    while True:
        print('watching subprocess')
        # reply = 'watching subprocess'
        # manage_json_packet(message_json, xbee, 'console', reply, True)
        # Read a line from the subprocess's output
        output_line = process.stdout.readline()
        

        if not output_line:
            # The subprocess closed its output and error, so exit the loop
            break
        
        # Parse the JSON string back to a Python object
        data_object = json.loads(output_line)

        # Convert the object to a MessagePack byte array
        packed_data = msgpack.packb(data_object)

        # Send the MessagePack data
        FT.sendXbee(xbee, packed_data)

    print('stopped watching subprocess')
    reply = 'stopped watching subprocess'
    manage_json_packet(message_json, xbee, 'console', reply, True)
    process.stdout.close()

    
def handle_stream_subprocess_error(process):
    while True:
        print('watching subprocess error')
        # reply = 'watching subprocess error'
        # manage_json_packet(message_json, xbee, 'console', reply, True)
        error_line = process.stderr.readline()
        if not error_line:
            break
        print("Error from subprocess: ", error_line)
        try:
            reply = "Error from subprocess: " + error_line
        except:
            reply = "Error from subprocess"
        manage_json_packet(message_json, xbee, 'console', reply, True)
    process.stderr.close()

# Start the command handling and listening threads
threading.Thread(target=handle_commands).start()
threading.Thread(target=listen_for_commands).start()  
print(os.path.abspath(__file__))
print('threads started')
reply = 'threads started'
manage_json_packet(message_json, xbee, 'console', reply, True)  
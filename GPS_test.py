#!/usr/bin/python3
import sys, threading, queue
import serial, string, datetime, time, Adafruit_ADS1x15, struct, adafruit_gps
import RPi.GPIO as GPIO
import numpy as np
import json

print('hello world')

# Set up GPS
GPS_serial = serial.Serial('/dev/ttySOFT0')
gps = adafruit_gps.GPS(GPS_serial, debug=True) 
gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
gps.send_command(b"PMTK220,1000")
last_print = time.monotonic()

print('gps module connected')

while True:
    # Get GPS update
    gps.update()
    if not gps.has_fix:
        gps_spd = 'N/A'
    if gps.speed_knots is not None:
        spdmps = gps.speed_knots*0.514444
        gps_spd = '{0:.2f}'.format(spdmps) # Conversion to m/s
    gps_fix = "{}".format(gps.fix_quality)
    print(gps_fix)
    print(gps_spd)
    print(gps)
    time.sleep(1)
# adapted from:
# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import pigpio
import time
import serial
import adafruit_bno055
import board
import busio
# uart = serial.Serial("/dev/serial0")
# sensor = adafruit_bno055.BNO055_UART(uart)

#RX_PIN = 21  # Set your chosen GPIO pin number for RX
#TX_PIN = 20  # Set your chosen GPIO pin number for TX

#pi = pigpio.pi()
#if not pi.connected:
#    print('line 13')
#    exit(0)

# Baud rate and other settings
#baud = 9600
#pi.set_mode(RX_PIN, pigpio.INPUT)
#pi.set_mode(TX_PIN, pigpio.OUTPUT)
#uart = pi.serial_open(RX_PIN, baud)
'''
on the same i2c their is a adc and in the python script it is connected like so
adc = Adafruit_ADS1x15.ADS1115()
GAIN = 2/3
where the gain is used later (idk why i copied it here)'''
#so base on the above this SHOULD work right????
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

last_val = 0xFFFF

def temperature():
    global last_val  # pylint: disable=global-statement
    result = sensor.temperature
    if abs(result - last_val) == 128:
        result = sensor.temperature
        if abs(result - last_val) == 128:
            return 0b00111111 & result
    last_val = result
    return result

try:
    while True:
        # print("Temperature: {} degrees C".format(sensor.temperature))
        
        print(
            "Temperature: {} degrees C".format(temperature())
        )  # Uncomment if using a Raspberry Pi
        
        print("Accelerometer (m/s^2): {}".format(sensor.acceleration))
        print("Magnetometer (microteslas): {}".format(sensor.magnetic))
        print("Gyroscope (rad/sec): {}".format(sensor.gyro))
        print("Euler angle: {}".format(sensor.euler))
        print("Quaternion: {}".format(sensor.quaternion))
        print("Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration))
        print("Gravity (m/s^2): {}".format(sensor.gravity))
        print()
        time.sleep(1)

except KeyboardInterrupt:
    pi.serial_close(serial)
    pi.stop()

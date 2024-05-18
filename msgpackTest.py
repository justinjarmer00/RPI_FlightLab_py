import msgpack
import serial
import time

def readXbee(xbee):
    try:
        if xbee.in_waiting > 0:
            message = xbee.readline().decode('utf-8').strip()
        else:
            message = 'none'
    except UnicodeDecodeError:
        message = 'error could_not_decode'
    return message

def sendXbee(xbee, reply):
    newline = b'\n'  # Byte representation of newline
    reply = newline + reply  # reply is already a byte array
    xbee.write(reply)

def manage_json_packet(queue, xbee, key, value, send=False):
    queue[key] = value

    if send:
        #packet = json.dumps(queue)
        packet = msgpack.packb(queue)
        sendXbee(xbee, packet)
        # print(packet, flush = True)  # Here, we're printing, but in the actual application, you'd send it over the desired medium.
        queue.clear()

xbee = serial.Serial(
    port ='/dev/ttyS0',  #/dev/ttyS0',
    baudrate = 115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)
print('made connection with Xbee')

while True:
    print(readXbee(xbee))
    time.sleep(0.5)
    message = b'hello!'
    sendXbee(xbee, message)
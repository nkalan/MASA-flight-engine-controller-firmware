
import serial
import serial.tools.list_ports
import struct



print("Ylol")

ser = serial.Serial(port='COM3', baudrate=int(115200), timeout=1.5)

if(ser.is_open):
    arr = list()
    for i in range(255):
        arr.append(i)

    # Send a packet
    while(1):
        ser.write(bytes(arr))
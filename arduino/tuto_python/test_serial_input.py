
import serial
import time

with serial.Serial('COM3', 9600) as ser:
    while(True):
        ser.write("abc".encode('utf-8'))
        time.sleep(0.1)

    """
    while(True):
        t = ser.read(100).decode("utf-8")
        line = t.split("\r\n")
        for w in line:
            print(w)
    """


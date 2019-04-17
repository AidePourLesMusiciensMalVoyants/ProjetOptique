import serial
import random

"""
Pour communiquer avec arduino, il faut inventer un protocole
Par exemple, ici on définit une syntaxe <x>|<y>#
pour envoyer deux nombres (x, y) qui représentent des coordonnées
"""

speed = 9600
## speed = 115200

last = ""

with serial.Serial('COM5', speed, timeout=.1) as arduino:
    while True:
        arduino.write("start#".encode("utf-8"))

        data = arduino.readline()
        print(data.strip())
        last = data

        for i in range(10):
            arduino.write("running#".encode("utf-8"))
            data = arduino.readline()
            print(data.strip())
            last = data

        arduino.write("stop#".encode("utf-8"))

        data = arduino.readline()
        print(data.strip())
        last = data

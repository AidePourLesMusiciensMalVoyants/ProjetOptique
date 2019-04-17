import serial
import random
import math
import sys, os, time

"""
Pour communiquer avec arduino, il faut inventer un protocole
Par exemple, ici on définit une syntaxe <x>|<y>#
pour envoyer deux nombres (x, y) qui représentent des coordonnées
"""

# speed = 9600
speed = 115200

def circle(t):
    x = int((math.sin(t) + 1.)*128)
    y = int((math.cos(t) + 1.)*128)

    #print(x, y)

    return x, y

def sinus(t):
    x = int((math.sin(t) + 1.)*128)
    z = int((t*10) / 256) % 2
    y = z*256 + (1 - 2*z)*int((t*10) % 256)

    # print(x, y, z)

    return x, y

with serial.Serial('COM5', speed, timeout=.1) as arduino:
    t = 0
    while True:
        x = int(random.random()*256)
        y = int(random.random()*256)
        # x, y = circle(t/100.0)
        x, y = sinus(t/50.0)

        arduino.write("{}|{}#".format(x, y).encode("utf-8"))

        data = arduino.readline()
        if data:
            print(">>>", data.strip())

        t += 1
        time.sleep(0.05)

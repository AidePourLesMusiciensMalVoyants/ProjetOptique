
import os, sys
sys.path.append('../application')

import math
import matplotlib.pyplot as plt
import random
import time
import serial

from Point import *

import re

HOST = 'nb-arnault4'
PORT = 5000

# speed = 9600
speed = 115200



def func():
    # return random.random()
    return math.sin(t)

def using_points():
    origin = time.time()

    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111)

    plotter = Plotter(fig, ax)
    pv_end = '(.+)'
    pv_in = '([^,]+)[,]'

    pv_ranges = pv_in*3 + pv_end + '$'
    pv_data = pv_in*4 + pv_end + '$'

    t0 = time.time()

    try:
        arduino = serial.Serial('COM5', speed, timeout=.1)
    except:
        arduino = None

    number = 0

    ##with open("../application/data.csv", "r") as f:
    with open("../application/Carnutes.csv", "r") as f:
        lines = f.readlines()
        first = True
        for line in lines:
            if first:
                first = False

                m = re.match(pv_ranges, line)
                if m is None:
                    continue

                min_x = float(m[1])
                max_x = float(m[2])
                min_y = float(m[3])
                max_y = float(m[4])

                left = plotter.add_point("left", 1, colorx="r", colory="g")
                left.set_x_range(min_x, max_x)
                left.set_y_range(min_y, max_y)

                right = plotter.add_point("right", 0, colorx="b", colory="y")
                right.set_x_range(min_x, max_x)
                right.set_y_range(min_y, max_y)

                plotter.start_plotting()

                continue

            m = re.match(pv_data, line)
            if m is None:
                continue

            t = float(m[1])
            x1 = float(m[2])
            y1 = float(m[3])
            x2 = float(m[4])
            y2 = float(m[5])

            plotter.plot(t - t0, [x1, x2], [1000 - y1, 1000 - y2])

            if not arduino is None:
                arduino.write("{}|{}|{}|{}|{}#".format(number, int(x1/8), int(y1/8), int(x2/8), int(y2/8)).encode("utf-8"))

                data = arduino.readline()
                if data:
                    print("received >>>", data.strip())
                number += 1

        # print("t=", t)

        # time.sleep(random.random()*0.1)

using_points()

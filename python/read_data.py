
import os, sys
sys.path.append('../application')

import math
import matplotlib.pyplot as plt
import random
import time
from Point import *
import re


def func():
    # return random.random()
    return math.sin(t)

def using_points():
    origin = time.time()

    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111)

    pv_end = '(.+)'
    pv_in = '([^,]+)[,]'

    pv_ranges = pv_in*3 + pv_end + '$'
    pv_data = pv_in*4 + pv_end + '$'

    with open("data.csv", "r") as f:
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

                p1 = Point("p1", 0)
                p1.set_x_range(min_x, max_x)
                p1.set_y_range(min_y, max_y)
                p2 = Point("p2", 1)
                p2.set_x_range(min_x, max_x)
                p2.set_y_range(min_y, max_y)

                p1.start_plotting(fig, ax)
                p2.start_plotting(fig, ax, colorx="b", colory="y")

                continue

            m = re.match(pv_data, line)
            if m is None:
                continue

            t = float(m[1])
            x1 = float(m[2])
            y1 = float(m[3])
            x2 = float(m[4])
            y2 = float(m[5])

            # print("t=", t)

            p1.set(t, x1, y1)
            p2.set(t, x2, y2)

            p1.plot()
            p2.plot()

        # time.sleep(random.random()*0.1)

using_points()

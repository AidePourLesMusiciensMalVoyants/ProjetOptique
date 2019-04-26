
import os, sys
sys.path.append('../application')

import math
import matplotlib.pyplot as plt
import random
import time
from Point import *


def func():
    # return random.random()
    return math.sin(t)

def using_points():
    origin = time.time()

    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111)

    plotter = Plotter(fig, ax)

    p = plotter.add_point("ppp")

    p.set_x_range(0, 2200)
    p.set_y_range(-300, 1300)
    p.set_v_range(-200000, 200000)

    plotter.start_plotting()

    while True:
        t = time.time()
        x = random.random()*2200
        y = random.random()*1600 - 300

        # print("t=", t)

        plotter.plot(t, [x], [y])

        # time.sleep(random.random()*0.1)

using_points()

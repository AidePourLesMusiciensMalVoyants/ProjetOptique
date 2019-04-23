
import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import random
import time
from pandas import Series
from pandas import DataFrame
from pandas import TimeGrouper
import pandas as pd
import datetime as dt
from Point import *

class Speed:
    def __init__(self, f, step):
        self.dt = 0
        self.prev = None
        self.value = None
        self.step = step
        self.old = None
        self.func = f

    def f(self, t):

        # x = math.sin(t)

        if self.dt == 0:
            if self.prev is None:
                self.prev = func()
                self.value = func()
                self.old = self.value

        vx = 0
        x = None
        if self.dt < self.step:
            vx = (self.value - self.prev) / self.step
            x = self.prev + (vx * self.dt)
            self.dt += 1
            # print(self.prev, self.dt, x, self.value)
        else:
            vx = (self.value - self.prev) / self.step
            self.prev = self.value
            x = self.value
            self.dt = 0
            # print(self.prev, x, self.value)
            self.value = func()

        self.old = self.value

        return x, vx*20

x = []
vx = []


def func():
    # return random.random()
    return math.sin(t)

def using_speed():
    a = Speed(func, 100.)

    t0 = time.time()
    for t in range(5000):
        t1 = time.time()

        print(t1 - t0)

        value, speed = a.f(t/90.)
        x.append(value)
        vx.append(speed)


    fig, ax = plt.subplots()
    ax.plot(x)
    ax.plot(vx)
    plt.show()

def using_points():
    origin = time.time()
    p = Point("p", rangex=[0.0, 1.0], rangey=[1.0, 2.0])

    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111)

    p.start_plotting(fig, ax)

    while True:

        x = random.random()
        y = random.random() + 1
        t = time.time() - origin

        # print("t=", t)

        t, x, y, vx, vy = p.set(t, x, y)

        p.plot()

        plt.pause(0.001)


using_points()

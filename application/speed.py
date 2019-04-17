
import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import random


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

a = Speed(func, 100.)

import time
from datetime import datetime

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


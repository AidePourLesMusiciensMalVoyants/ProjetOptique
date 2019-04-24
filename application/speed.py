
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


def func():
    # return random.random()
    return math.sin(t)

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

        # time.sleep(random.random()*0.1)

using_points()

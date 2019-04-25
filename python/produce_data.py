
import os, sys
sys.path.append('../application')

import math
import random
import time
import signal
import sys


"""
print('Press Ctrl+C')
signal.pause()
"""

stopped = False

def func():
    # return random.random()
    return math.sin(t)

def signal_handler(sig, frame):
    global stopped
    print('You pressed Ctrl+C!')
    stopped = True

def produce():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    origin = time.time()

    """
    min_x = 0
    max_x = 2200
    min_y = -300
    max_y = 1300
    """

    min_x = 0
    max_x = 1000
    min_y = -5000
    max_y = 5000

    with open("data.csv", "w+") as f:
        f.write("{}, {}, {}, {}\n".format(min_x, max_x, min_y, max_y))
        for n in range(10000):
            t = time.time() - origin
            x1 = random.random()*(max_x - min_x) + min_x
            y1 = random.random()*(max_y - min_y) + min_y
            x2 = random.random()*(max_x - min_x) + min_x
            y2 = random.random()*(max_y - min_y) + min_y
            f.write("{}, {}, {}, {}, {}\n".format(t, x1, y1, x2, y2))
            time.sleep(random.random()*0.1)

            if stopped:
                break

produce()

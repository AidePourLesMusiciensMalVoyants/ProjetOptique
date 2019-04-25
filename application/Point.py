
import math
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline, BSpline
from scipy.ndimage.filters import gaussian_filter1d


"""
Class Point 
Handle coordinates of a point in space
Acquire realtime values of x, y
Compute the speed (vx,vy) = (dx/dt, dy/dt)
May plot the evolution of x & y on a dynamic plot
"""
class Point(object):
    def __init__(self, name, order):
        self.name = name            # general key
        self.order = order
        self.first = True           # general start marker
        self.ranges = dict()        # declared or computed ranges

        self.t = None               # acquisition
        self.x = None
        self.y = None

        self.vx = None              # computation
        self.vy = None

        self.sx = None              # scaled values
        self.sy = None
        self.svx = None
        self.svy = None

        self.prevt = None           # differences to compute the speed
        self.prevx = None
        self.prevy = None

        # plotting the point data
        self.nbins = 500
        self.ts = [0] * self.nbins  # buffered data for plotting
        self.xs = [0] * self.nbins
        self.ys = [0] * self.nbins

        self.plotting = False       # state for plotting
        self.origin = None          # origin of acquisition
        self.plot_index = None      # current position in plotting buffer
        self.fig = None             # matplotlib variables
        self.ax = None
        self.plot_first = True
        self.xline = None           # graphical objects
        self.yline = None

        self.min_x = None
        self.max_x = None
        self.min_y = None
        self.max_y = None
        self.min_v = None
        self.max_v = None

        self.scale_factor = 2048.0

    def set_x_range(self, min_x, max_x):
        self.min_x = min_x
        self.max_x = max_x

    def set_y_range(self, min_y, max_y):
        self.min_y = min_y
        self.max_y = max_y

    def set_v_range(self, min_v, max_v):
        self.min_v = min_v
        self.max_v = max_v

    def set(self, t, x, y):
        """
        Real time acquisition of position
        Compute speed
        """
        while self.first:
            self.first = False

            self.prevt = t
            self.prevx = x
            self.prevy = x

            continue

        self.t = t
        self.x = x
        self.y = y

        dt = t - self.prevt
        dx = x - self.prevx
        dy = y - self.prevy

        if math.isinf(x):
            print("x=", x, " y=", y)
        if math.isinf(y):
            print("x=", x, " y=", y)

        try:
            self.vx = dx/dt
            self.vy = dy/dt
        except:
            self.vx = 0.0
            self.vy = 0.0

        self.scale()

        # print("set>>> t={} x={} y={} vx={} vy={}".format(t, x, y, self.vx, self.vy))

        return self.t, self.x, self.y, self.vx, self.vy


    def range(self):
        """
        Adapt the range
        """
        for key in ["x", "y", "vx", "vy"]:
            if key == "x":
                value = self.x
            elif key == "y":
                value = self.y
            elif key == "vx":
                value = self.vx
            elif key == "vy":
                value = self.vy

            try:
                value = int(value)
            except:
                continue

            if key == "x":
                if self.min_x is None or value < self.min_x: self.min_x = value
                if self.max_x is None or value > self.max_x: self.max_x = value
            elif key == "y":
                if self.min_y is None or value < self.min_y: self.min_y = value
                if self.max_y is None or value > self.max_y: self.max_y = value
            elif key == "vx"or key == "vy":
                if self.min_v is None or value < self.min_v: self.min_v = value
                if self.max_v is None or value > self.max_v: self.max_v = value

    def print_range(self):
        t = ""

        t += " x=[{} .. {}] ".format(self.min_x, self.max_x)
        t += " y=[{} .. {}] ".format(self.min_y, self.max_y)
        t += " v=[{} .. {}] ".format(self.min_v, self.max_v)
        return t

    def scale(self):
        """
        Compute scaled values according to range
        """
        def compute_scale(value, vmin, vmax):
            if not vmin is None and value < vmin: value = vmin
            if not vmax is None and value > vmax: value = vmax
            scaled = 0
            try:
                value = int(value)
                a = float(value - vmin)
                b = float(vmax - vmin)
                scaled = int(float(a/b) * self.scale_factor)
            except:
                pass
            return scaled

        for key in ["x", "y", "vx", "vy"]:
            if not key in self.ranges:
                self.ranges[key] = {'min': None, 'max': None}

            r = self.ranges[key]

            if key == "x":
                value = self.x
                vmin = self.min_x
                vmax = self.max_x
            elif key == "y":
                value = self.y
                vmin = self.min_y
                vmax = self.max_y
            elif key == "vx":
                value = self.vx
                vmin = self.min_v
                vmax = self.max_v
            elif key == "vy":
                value = self.vy
                vmin = self.min_v
                vmax = self.max_v

            if value is math.inf or value is None or value is math.nan:
                value = None
                print("infini")

            if not (value is None or value is math.nan):
                svalue = compute_scale(value, vmin, vmax)

                if key == "x":
                    self.sx = svalue
                elif key == "y":
                    self.sy = svalue
                elif key == "vx":
                    self.svx = svalue
                elif key == "vy":
                    self.svy = svalue

    def start_plotting(self, fig, ax, colorx="r", colory="g"):
        """
        Initialize the plotting
        """
        self.plotting = True
        self.origin = time.time()
        self.plot_index = 0
        self.fig = fig
        self.ax = ax

        self.plot_first = True
        self.xline = None
        self.yline = None
        self.colorx = colorx
        self.colory = colory

        # initialize default extrema according to range
        if 'x' in self.ranges and 'y' in self.ranges:
            self.set(time.time(), self.min_x, self.min_y)
            self.set(time.time(), self.max_x, self.max_y)

    def plot(self):
        """
        Dynamic plotting of acquired values
        1)
            fill in one buffer of values before really plotting
        2)
            once the buffer is filled, coming values are appended at end of cyclic buffer

        At first real plot action, plotting objects are created and time line is created linear
        Then for every plot plotting objects are refreshed from real scaled data

        """
        if not self.plotting:
            return

        t = self.t - self.origin
        x = self.sx + 2*self.scale_factor*self.order
        y = self.sy + 2*self.scale_factor*self.order + self.scale_factor

        if self.plot_index < self.nbins:
            # just filling the buffer. No real plot
            self.ts[self.plot_index] = t
            self.xs[self.plot_index] = x
            self.ys[self.plot_index] = y

            self.plot_index += 1

            self.i = 0

            return

        # now the buffer is filled , we inject new data to the buffer as a cyclic buffer
        self.i += 1

        self.ts.pop(0)
        self.xs.pop(0)
        self.ys.pop(0)
        self.ts.append(t)
        self.xs.append(x)
        self.ys.append(y)

        # synchronize times to moving origin
        t0 = self.ts[0]
        time_line = [(t - t0) for t in self.ts]

        smoothing_factor = 1.0

        if self.plot_first:
            # initialization of plotting objects
            self.plot_first = False

            """
            we initialize the time line, with a linear distribution of times to erase irregular starting times
            """
            time_line = np.linspace(0.0, self.nbins/20.0, self.nbins)

            """
            to initialize the data graphs, we setup a linear distribution from min to max
            and we position x & y separately according to the "order" specification
            """
            x_initializer = np.linspace(0.0,
                                        self.scale_factor,
                                        self.nbins)
            y_initializer =  np.linspace(2*self.scale_factor*self.order + self.scale_factor,
                                         2*self.scale_factor*self.order + 2*self.scale_factor,
                                         self.nbins)

            self.xline, = self.ax.plot(time_line, x_initializer, '{}-'.format(self.colorx), label=self.name + '_x')
            self.yline, = self.ax.plot(time_line, y_initializer, '{}-'.format(self.colory), label=self.name + '_y')
            self.ax.legend(loc='upper left', shadow=True)
        else:
            xs_smoothed = gaussian_filter1d(self.xs, sigma=smoothing_factor)
            ys_smoothed = gaussian_filter1d(self.ys, sigma=smoothing_factor)

            self.xline.set_xdata(time_line)
            # self.xline.set_ydata(self.xs)
            self.xline.set_ydata(xs_smoothed)
            self.yline.set_xdata(time_line)
            # self.yline.set_ydata(self.ys)
            self.yline.set_ydata(ys_smoothed)

        plt.pause(0.0001)

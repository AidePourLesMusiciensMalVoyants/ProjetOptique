
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
    def __init__(self, name, rangex=None, rangey=None, rangev=None):
        self.name = name            # general key
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

        if not rangex is None:
            self.ranges["x"] = {'min': rangex[0], 'max': rangex[1]}
        if not rangey is None:
            self.ranges["y"] = {'min': rangey[0], 'max': rangey[1]}
        if not rangev is None:
            self.ranges["vx"] = {'min': rangev[0], 'max': rangev[1]}
            self.ranges["vy"] = {'min': rangev[0], 'max': rangev[1]}

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

        try:
            self.vx = dx/dt
            self.vy = dy/dt
        except:
            self.vx = 0.0
            self.vy = 0.0

        # print("set>>> t={} x={} y={} vx={} vy={}".format(t, x, y, self.vx, self.vy))

        self.range()
        text = self.print_range()
        # print("ranges ", self.name, text)

        return self.t, self.x, self.y, self.vx, self.vy


    def range(self):
        """
        Compute the range when not specified at creation
        :return:
        """
        for key in ["x", "y", "vx", "vy"]:
            if not key in self.ranges:
                self.ranges[key] = {'min': None, 'max': None}

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

            r = self.ranges[key]

            if (r['min'] is None) or (value < r['min']):
                r['min'] = value
            if (r['max'] is None) or (value > r['max']):
                r['max'] = value

    def print_range(self):
        t = ""
        for key in ["x", "y", "vx", "vy"]:
            if not key in self.ranges:
                self.ranges[key] = {'min': None, 'max': None}
            r = self.ranges[key]

            t += " {} -> min={} max={}".format(key, r['min'], r['max'])
        return t

    def scale(self):
        """
        Compute scaled values according to range
        """
        def compute_scale(value, vmin, vmax):
            if value < vmin: value = vmin
            if value > vmax: value = vmax
            scaled = 0
            try:
                value = int(value)
                a = float(value - vmin)
                b = float(vmax - vmin)
                scaled = int(float(a/b)*256)
            except:
                pass
            return scaled

        for key in ["x", "y", "vx", "vy"]:
            if not key in self.ranges:
                self.ranges[key] = {'min': None, 'max': None}

            r = self.ranges[key]

            if key == "x":
                value = self.x
            elif key == "y":
                value = self.y
            elif key == "vx":
                value = self.vx
            elif key == "vy":
                value = self.vy

            if not (value is None or value is math.nan):
                svalue = compute_scale(value, r['min'], r['max'])

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
        self.set(time.time(), self.ranges["x"]["min"], self.ranges["y"]["min"])
        self.set(time.time(), self.ranges["x"]["max"], self.ranges["y"]["max"])

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
        x = self.x
        y = self.y

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

        """

            new_time_line = np.linspace(time_line.min(), time_line.max(), self.nbins)  # 300 represents number of points to make between T.min and T.max

            spl = make_interp_spline(time_line, power, k=3)  # BSpline object
            power_smooth = spl(xnew)

plt.plot(xnew,power_smooth)
        """

        smooth_factor = 1.0

        if self.plot_first:
            # initialization of plotting objects
            self.plot_first = False

            """
            we initialize the time line, with a linear distribution of times to erase irregular starting times
            """
            time_line = np.linspace(0.0, 5.0, self.nbins)

            xs_smoothed = gaussian_filter1d(self.xs, sigma=smooth_factor)
            ys_smoothed = gaussian_filter1d(self.ys, sigma=smooth_factor)

            self.xline, = self.ax.plot(time_line, xs_smoothed, '{}-'.format(self.colorx), label=self.name + '_x')
            self.yline, = self.ax.plot(time_line, ys_smoothed, '{}-'.format(self.colory), label=self.name + '_y')
            self.ax.legend(loc='upper left', shadow=True)
        else:
            xs_smoothed = gaussian_filter1d(self.xs, sigma=smooth_factor)
            ys_smoothed = gaussian_filter1d(self.ys, sigma=smooth_factor)

            self.xline.set_xdata(time_line)
            self.xline.set_ydata(xs_smoothed)
            self.yline.set_xdata(time_line)
            self.yline.set_ydata(ys_smoothed)

        plt.pause(0.001)

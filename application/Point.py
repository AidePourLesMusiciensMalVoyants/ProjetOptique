
import math
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline, BSpline
from scipy.ndimage.filters import gaussian_filter1d


pt = None
px = None
py = None

class PlotLine(object):
    def __init__(self, nbins, point, order, colorx="r", colory="g"):
        self.nbins = nbins
        self.point = point
        self.order = order
        self.colorx = colorx
        self.colory = colory
        self.xline = None
        self.yline = None

        self.xs = []
        self.ys = []

    def set_plot_lines(self, xline, yline):
        self.xline = xline
        self.yline = yline

    def put(self, x, y):
        if len(self.xs) == self.nbins:
            self.xs.pop(0)
            self.ys.pop(0)
        self.xs.append(x)
        self.ys.append(y)


class Plotter(object):
    def __init__(self, fig, ax):
        self.nbins = 200.
        self.scale_factor = 2048.
        self.fig = fig
        self.ax = ax
        self.lines = []
        self.origin = time.time()
        self.plotting = False
        self.ts = []
        self.started = False


    def add_point(self, name, order=0, colorx="r", colory="g"):
        point = Point(name, self)
        line = PlotLine(self.nbins, point, order, colorx, colory)
        self.lines.append(line)
        return point

    def start_plotting(self):
        global px, py

        print("start_plotting")

        time_line = np.linspace(0.0, self.nbins / 20.0, self.nbins)

        """
        to initialize the data graphs, we setup a linear distribution from min to max
        and we position x & y separately according to the "order" specification
        """
        for line in self.lines:
            offset = 2 * self.scale_factor * line.order
            x_min = offset
            x_max = offset + self.scale_factor
            x_initializer = np.linspace(x_min, x_max, self.nbins)

            y_min = offset + self.scale_factor
            y_max = offset + 2 * self.scale_factor
            y_initializer = np.linspace(y_min, y_max, self.nbins)

            px, = self.ax.plot(time_line, x_initializer, '{}-'.format(line.colorx), label=line.point.name + '_x')
            py, = self.ax.plot(time_line, y_initializer, '{}-'.format(line.colory), label=line.point.name + '_y')

            line.set_plot_lines(px, py)

        self.ax.legend(loc='upper left', shadow=True)

        self.plotting = True

    def plot(self, t, xs, ys):
        if not self.plotting:
            return

        t = t - self.origin

        if len(self.ts) == self.nbins:
            self.ts.pop(0)

        self.ts.append(t)

        x1 = None
        y1 = None
        x2 = None
        y2 = None

        for i, line in enumerate(self.lines):
            x = xs[i]
            y = ys[i]
            t, x, y, _, _ = line.point.set(t, x, y)
            if i == 0:
                x1 = x
                y1 = y
            if i == 1:
                x2 = x
                y2 = y

            # print(x, y)
            line.put(x, y)

        if self.started:
            # xs_smoothed = gaussian_filter1d(self.xs, sigma=smoothing_factor)
            # ys_smoothed = gaussian_filter1d(self.ys, sigma=smoothing_factor)

            ttt = [t - self.ts[0] for t in self.ts]

            for line in self.lines:
                offsetx = 2 * self.scale_factor * line.order
                offsety = 2 * self.scale_factor * line.order + self.scale_factor

                line.xline.set_xdata(ttt)
                line.xline.set_ydata([x + offsetx for x in line.xs])

                line.yline.set_xdata(ttt)

                line.yline.set_ydata([y + offsety for y in line.ys])

                self.fig.gca().relim()
                self.fig.gca().autoscale_view()
                self.fig.canvas.draw()
                self.fig.canvas.flush_events()

        elif len(self.ts) == self.nbins:
            self.started = True

        return t, x1, y1, x2, y2



"""
Class Point 
Handle coordinates of a point in space
Acquire realtime values of x, y
Compute the speed (vx,vy) = (dx/dt, dy/dt)
May plot the evolution of x & y on a dynamic plot
"""
class Point(object):
    def __init__(self, name, plotter):
        self.name = name            # general key
        self.first = True           # general start marker

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
        self.nbins = plotter.nbins

        self.min_x = None
        self.max_x = None
        self.min_y = None
        self.max_y = None
        self.min_v = None
        self.max_v = None

        self.scale_factor = plotter.scale_factor

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

        """
        if math.isinf(x):
            print("x=", x, " y=", y)
        if math.isinf(y):
            print("x=", x, " y=", y)
        """

        try:
            self.vx = dx/dt
            self.vy = dy/dt
        except:
            self.vx = 0.0
            self.vy = 0.0

        self.scale()

        # print("set>>> t={} x={} y={} vx={} vy={}".format(t, x, y, self.vx, self.vy))

        return self.t, self.sx, self.sy, self.svx, self.svy


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


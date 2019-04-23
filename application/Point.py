
import math
import time
import numpy as np
import matplotlib.pyplot as plt


class Point(object):
    def __init__(self, name, rangex=None, rangey=None, rangev=None):
        self.name = name
        self.first = True
        self.ranges = dict()
        self.t = None
        self.x = None
        self.y = None
        self.vx = None
        self.vy = None
        self.sx = None
        self.sy = None
        self.svx = None
        self.svy = None
        self.prevt = None
        self.prevx = None
        self.prevy = None

        # plotting the oint data
        self.nbins = 200
        self.ts = [0] * self.nbins
        self.xs = [0] * self.nbins
        self.ys = [0] * self.nbins

        self.plotting = False
        self.origin = None
        self.plot_index = None
        self.fig = None
        self.ax = None
        self.plot_first = True
        self.xline = None
        self.yline = None

        if not rangex is None:
            self.ranges["x"] = {'min': rangex[0], 'max': rangex[1]}
        if not rangey is None:
            self.ranges["y"] = {'min': rangey[0], 'max': rangey[1]}
        if not rangev is None:
            self.ranges["vx"] = {'min': rangev[0], 'max': rangev[1]}
            self.ranges["vy"] = {'min': rangev[0], 'max': rangev[1]}

    def set(self, t, x, y):
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

        self.set(time.time(), self.ranges["x"]["min"], self.ranges["y"]["min"])
        self.set(time.time(), self.ranges["x"]["max"], self.ranges["y"]["max"])

    def plot(self):
        if not self.plotting:
            return

        t = self.t - self.origin
        x = self.x
        y = self.y

        if self.plot_index < self.nbins:
            self.ts[self.plot_index] = t
            self.xs[self.plot_index] = x
            self.ys[self.plot_index] = y

            self.plot_index += 1

            return

        self.ts.pop(0)
        self.xs.pop(0)
        self.ys.pop(0)
        self.ts.append(t)
        self.xs.append(x)
        self.ys.append(y)

        #x0 = self.ts[0]
        #ttt = [(t - x0) for t in self.ts[0:self.plot_index + 1]]

        if self.plot_index == self.nbins:

            if self.plot_first:
                self.plot_first = False

                x = np.linspace(0, 1, self.nbins)
                self.xline, = self.ax.plot(x, self.xs, '{}-'.format(self.colorx))
                self.yline, = self.ax.plot(x, self.ys, '{}-'.format(self.colory))
            else:
                # self.xline.set_xdata(self.xs)
                self.xline.set_ydata(self.xs)
                self.yline.set_ydata(self.ys)

        plt.pause(0.001)

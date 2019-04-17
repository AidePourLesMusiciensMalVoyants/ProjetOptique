from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

import ctypes
import _ctypes
import pygame
import sys
import serial
import random
import math
import sys, os, time

if sys.hexversion >= 0x03000000:
    import _thread as thread
else:
    import thread

speed = 115200

# colors for drawing different bodies 
SKELETON_COLORS = [pygame.color.THECOLORS["red"], 
                  pygame.color.THECOLORS["blue"], 
                  pygame.color.THECOLORS["green"], 
                  pygame.color.THECOLORS["orange"], 
                  pygame.color.THECOLORS["purple"], 
                  pygame.color.THECOLORS["yellow"], 
                  pygame.color.THECOLORS["violet"]]


x_min = None
x_max = None
y_min = None
y_max = None
z_min = None
z_max = None

xx_min = None
xx_max = None
yy_min = None
yy_max = None

import socket, sys
import random
import time

HOST = 'nb-arnault4'
PORT = 5000

mySocket = None

message = 0

OK = 0
DISCONNEXION = 1
QUIT = 2
ERROR = 3

def set_connexion():

    print("testing for connexion...")

    # 1) création du socket :
    try:
        mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    except socket.error:
        return ERROR, None

    # 2) envoi d'une requête de connexion au serveur :
    try:
        mySocket.connect((HOST, PORT))
    except socket.error:
        return ERROR, None

    print("Connexion établie avec le serveur.")

    # 3) Dialogue avec le serveur :

    return OK, mySocket


def send_data(id1, x1, y1, id2, x2, y2):
    global message
    global mySocket

    if mySocket is None:
        status, mySocket = set_connexion()
        if status != OK:
            return status

    try:
        mySocket.send("{}|{}|{}|{}|{}|{}#".format(id1, x1, y1, id2, x2, y2).encode("utf-8"))
        message += 1
    except socket.error:
        print("Disconnexion from server")
        mySocket = None
        return DISCONNEXION

    try:
        # time.sleep(0.0001)
        for i in range(1000):
            pass
        pass
    except KeyboardInterrupt:
        mySocket = None
        return QUIT

    return OK



class BodyGameRuntime(object):
    def __init__(self):
        self.ranges = dict()

        pygame.init()

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Set the width and height of the screen [width, height]
        self._infoObject = pygame.display.Info()
        self._screen = pygame.display.set_mode((self._infoObject.current_w >> 1, self._infoObject.current_h >> 1), 
                                               pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE, 32)

        pygame.display.set_caption("Kinect for Windows v2 Body Game")

        # Loop until the user clicks the close button.
        self._done = False

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Kinect runtime object, we want only color and body frames 
        self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body)

        # back buffer surface for getting Kinect color frames, 32bit color, width and height equal to the Kinect color frame size
        self._frame_surface = pygame.Surface((self._kinect.color_frame_desc.Width, self._kinect.color_frame_desc.Height), 0, 32)

        # here we will store skeleton data 
        self._bodies = None


    def draw_body_bone(self, joints, jointPoints, depthPoints, color, joint0, joint1):
        global x_min, x_max
        global y_min, y_max
        global z_min, z_max
        global xx_min, xx_max
        global yy_min, yy_max

        joint0State = joints[joint0].TrackingState;
        joint1State = joints[joint1].TrackingState;

        # both joints are not tracked
        if (joint0State == PyKinectV2.TrackingState_NotTracked) or (joint1State == PyKinectV2.TrackingState_NotTracked): 
            return

        # both joints are not *really* tracked
        if (joint0State == PyKinectV2.TrackingState_Inferred) and (joint1State == PyKinectV2.TrackingState_Inferred):
            return

        # ok, at least one is good 
        start = (jointPoints[joint0].x, jointPoints[joint0].y)
        end = (jointPoints[joint1].x, jointPoints[joint1].y)

        a0 = jointPoints[joint0]
        a1 = jointPoints[joint1]
        # print(a0, a1)

        for ji in joints._objects:
            # print(ji)
            os = joints._objects[ji]
            io = os[11]
            pos = io.Position

            if x_min is None:
                x_min = pos.x
            elif pos.x < x_min:
                x_min = pos.x

            if y_min is None:
                y_min = pos.y
            elif pos.y < y_min:
                y_min = pos.y

            if z_min is None:
                z_min = pos.z
            elif pos.z < z_min:
                z_min = pos.z

            if x_max is None:
                x_max = pos.x
            elif pos.x > x_max:
                x_max = pos.x

            if y_max is None:
                y_max = pos.y
            elif pos.y > y_max:
                y_max = pos.y

            if z_max is None:
                z_max = pos.z
            elif pos.z > z_max:
                z_max = pos.z

            if xx_min is None:
                xx_min = a0.x
            elif a0.x < xx_min:
                xx_min = a0.x

            if yy_min is None:
                yy_min = a0.y
            elif a0.y < yy_min:
                yy_min = a0.y

            if xx_max is None:
                xx_max = a0.x
            elif a0.x > xx_max:
                xx_max = a0.x

            if yy_max is None:
                yy_max = a0.y
            elif a0.y > yy_max:
                yy_max = a0.y

            # print("x= [", x_min, ",", x_max, "] y=[", y_min, ",", y_max, "] z=[", z_min, ",", z_max, "] xx= [", xx_min, ",", xx_max, "] yy=[", yy_min, ",", yy_max, "]")

        # print("Color={} from [{:3f}, {:3f}] to [{:3f}, {:3f}] ".format(color, a0.x, a0.y, a1.x, a1.y))

        # with serial.Serial('COM5', speed, timeout=.1) as arduino:
        #    pass

        try:
            pygame.draw.line(self._frame_surface, color, start, end, 8)
        except: # need to catch it due to possible invalid positions (with inf)
            pass

    def range(self, key, value):
        if not key in self.ranges:
            self.ranges[key] = {'min': None, 'max': None}

        try:
            value = int(value)
        except:
            value = sys.maxsize

        r = self.ranges[key]

        if (r['min'] is None) or (value < r['min']):
            r['min'] = value
        if (r['max'] is None) or (value > r['max']):
            r['max'] = value

    def print_range(self, key):
        if not key in self.ranges:
            self.ranges[key] = {'min': None, 'max': None}
        r = self.ranges[key]

        print("{} -> min={} max={}".format(key, r['min'], r['max']))

    def scale(self, value, vmin, vmax):
        if value < vmin: value = vmin
        if value > vmax: value = vmax
        value = int(value)
        a = float(value - vmin)
        b = float(vmax - vmin)
        scaled = int(float(a/b)*256)
        return scaled

    def draw_body(self, joints, jointPoints, depthPoints, color):
        # Torso
        """
        self.draw_body_bone(joints, jointPoints, depthPoints, color, PyKinectV2.JointType_Head, PyKinectV2.JointType_Neck);
        self.draw_body_bone(joints, jointPoints, depthPoints, color, PyKinectV2.JointType_Neck, PyKinectV2.JointType_SpineShoulder);
        self.draw_body_bone(joints, jointPoints, depthPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_SpineMid);
        self.draw_body_bone(joints, jointPoints, depthPoints, color, PyKinectV2.JointType_SpineMid, PyKinectV2.JointType_SpineBase);
        self.draw_body_bone(joints, jointPoints, depthPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_ShoulderRight);
        self.draw_body_bone(joints, jointPoints, depthPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_ShoulderLeft);
        self.draw_body_bone(joints, jointPoints, depthPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipRight);
        self.draw_body_bone(joints, jointPoints, depthPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipLeft);
        """
    
        # Left Arm
        self.draw_body_bone(joints, jointPoints, depthPoints, color, PyKinectV2.JointType_ShoulderLeft, PyKinectV2.JointType_ElbowLeft);
        self.draw_body_bone(joints, jointPoints, depthPoints, color, PyKinectV2.JointType_ElbowLeft, PyKinectV2.JointType_WristLeft);
        self.draw_body_bone(joints, jointPoints, depthPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_HandLeft);
        self.draw_body_bone(joints, jointPoints, depthPoints, color, PyKinectV2.JointType_HandLeft, PyKinectV2.JointType_HandTipLeft);
        self.draw_body_bone(joints, jointPoints, depthPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_ThumbLeft);

        # Right Arm
        self.draw_body_bone(joints, jointPoints, depthPoints, color, PyKinectV2.JointType_ShoulderRight, PyKinectV2.JointType_ElbowRight);
        self.draw_body_bone(joints, jointPoints, depthPoints, color, PyKinectV2.JointType_ElbowRight, PyKinectV2.JointType_WristRight);
        self.draw_body_bone(joints, jointPoints, depthPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_HandRight);
        self.draw_body_bone(joints, jointPoints, depthPoints, color, PyKinectV2.JointType_HandRight, PyKinectV2.JointType_HandTipRight);
        self.draw_body_bone(joints, jointPoints, depthPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_ThumbRight);

        PyKinectV2.JointType_HandRight
        right = jointPoints[PyKinectV2.JointType_HandRight]
        left = jointPoints[PyKinectV2.JointType_HandLeft]

        """
        x [0 .. 2000]
        y [0 .. 1500]
        """

        self.range("xright", right.x)
        self.range("yright", right.y)

        self.range("xleft", left.x)
        self.range("yleft", left.y)

        """
        self.print_range("xright")
        self.print_range("yright")
        self.print_range("xleft")
        self.print_range("yleft")
        """

        rx = self.scale(right.x, 0, 2000)
        ry = self.scale(right.y, 0, 1500)
        lx = self.scale(left.x, 0, 2000)
        ly = self.scale(left.y, 0, 1500)

        ### print("right = [{} {}] left = [{} {}] right = [{} {}] left = [{} {}]".format(right.x, right.y, left.x, left.y, rx, ry, lx, ly))

        print("rx={} ry={} lx={} ly{}".format(rx, ry, lx, ly))

        status = send_data(1, rx, ry, 2, lx, ly)

        # Right Leg
        """
        self.draw_body_bone(joints, jointPoints, depthPoints, color, PyKinectV2.JointType_HipRight, PyKinectV2.JointType_KneeRight);
        self.draw_body_bone(joints, jointPoints, depthPoints, color, PyKinectV2.JointType_KneeRight, PyKinectV2.JointType_AnkleRight);
        self.draw_body_bone(joints, jointPoints, depthPoints, color, PyKinectV2.JointType_AnkleRight, PyKinectV2.JointType_FootRight);
        """

        # Left Leg
        """
        self.draw_body_bone(joints, jointPoints, depthPoints, color, PyKinectV2.JointType_HipLeft, PyKinectV2.JointType_KneeLeft);
        self.draw_body_bone(joints, jointPoints, depthPoints, color, PyKinectV2.JointType_KneeLeft, PyKinectV2.JointType_AnkleLeft);
        self.draw_body_bone(joints, jointPoints, depthPoints, color, PyKinectV2.JointType_AnkleLeft, PyKinectV2.JointType_FootLeft);
        """


    def draw_color_frame(self, frame, target_surface):
        target_surface.lock()
        address = self._kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame.ctypes.data, frame.size)
        del address
        target_surface.unlock()

    def run(self):
        global mySocket

        """
        while True:
            status, mySocket = set_connexion()
            if status == OK:
                break
        """

        quit = False
        status = OK

        # -------- Main Program Loop -----------
        # print("run")
        while not self._done:
            # --- Main event loop
            # print("run")
            for event in pygame.event.get(): # User did something
                if event.type == pygame.QUIT: # If user clicked close
                    self._done = True # Flag that we are done so we exit this loop

                elif event.type == pygame.VIDEORESIZE: # window resized
                    self._screen = pygame.display.set_mode(event.dict['size'], 
                                               pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE, 32)
                    
            # --- Game logic should go here

            # --- Getting frames and drawing  
            # --- Woohoo! We've got a color frame! Let's fill out back buffer surface with frame's data 
            if self._kinect.has_new_color_frame():
                frame = self._kinect.get_last_color_frame()
                self.draw_color_frame(frame, self._frame_surface)
                frame = None

            # --- Cool! We have a body frame, so can get skeletons
            if self._kinect.has_new_body_frame(): 
                self._bodies = self._kinect.get_last_body_frame()

            # --- draw skeletons to _frame_surface
            if self._bodies is not None: 
                for i in range(0, self._kinect.max_body_count):
                    body = self._bodies.bodies[i]
                    if not body.is_tracked: 
                        continue 
                    
                    joints = body.joints 
                    # convert joint coordinates to color space

                    joint_points = self._kinect.body_joints_to_color_space(joints)
                    depth_points = self._kinect.body_joints_to_depth_space(joints)
                    self.draw_body(joints, joint_points, depth_points, SKELETON_COLORS[i])

            # --- copy back buffer surface pixels to the screen, resize it if needed and keep aspect ratio
            # --- (screen size may be different from Kinect's color frame size) 
            h_to_w = float(self._frame_surface.get_height()) / self._frame_surface.get_width()
            target_height = int(h_to_w * self._screen.get_width())
            surface_to_draw = pygame.transform.scale(self._frame_surface, (self._screen.get_width(), target_height));
            self._screen.blit(surface_to_draw, (0,0))
            surface_to_draw = None
            pygame.display.update()

            # --- Go ahead and update the screen with what we've drawn.
            pygame.display.flip()

            # --- Limit to 60 frames per second
            self._clock.tick(60)

        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()
        pygame.quit()


__main__ = "Kinect v2 Body Game"
game = BodyGameRuntime();
game.run();


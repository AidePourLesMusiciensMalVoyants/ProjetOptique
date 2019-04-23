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
from Point import *
import numpy as np
import matplotlib.pyplot as plt

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

plot_origin = time.time()

plt.ion()
plot_fig = plt.figure()
plot_ax = plot_fig.add_subplot(111)


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


def send_data(x1, y1, x2, y2):
    global message
    global mySocket

    if mySocket is None:
        status, mySocket = set_connexion()
        if status != OK:
            return status

    try:
        mySocket.send("{}|{}|{}|{}#".format(x1, y1, x2, y2).encode("utf-8"))
        message += 1
    except socket.error:
        print("Disconnexion from server")
        mySocket = None
        return DISCONNEXION

    try:
        # time.sleep(0.0001)
        for i in range(100):
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

        self.t0 = time.time()
        self.t = time.time()

        """
        x [0 .. 2000]
        y [0 .. 1500]
        vx [-200000 .. 200000]
        """

        self.left = Point("left", (0, 2000), (0, 2000), (-200000, 200000))
        self.right = Point("right", (0, 2000), (0, 2000), (-200000, 200000))

        self.left.start_plotting(plot_fig, plot_ax, colorx="r", colory="g")
        self.right.start_plotting(plot_fig, plot_ax, colorx="b", colory="y")



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

        try:
            pygame.draw.line(self._frame_surface, color, start, end, 8)
        except: # need to catch it due to possible invalid positions (with inf)
            pass

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
        y [0 .. 2000]
        vx [-200000 .. 200000]
        """

        t = time.time()
        self.left.set(t, left.x, 1000 - left.y)
        self.right.set(t, right.x, 1000 - right.y)

        self.left.scale()
        self.right.scale()

        self.left.plot()
        self.right.plot()

        """
        print("Positions>>> rx={} ry={} lx={} ly={} rvx={} rvy={} lvx={} lvy={}".format(self.left.sx,
                                                                                        self.left.sy,
                                                                                        self.right.sx,
                                                                                        self.right.sy,
                                                                                        self.left.svx,
                                                                                        self.left.svy,
                                                                                        self.right.svx,
                                                                                        self.right.svy
                                                                                        ))
        """

        status = send_data(self.left.sx, self.left.sy, self.right.sx, self.right.sy)

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


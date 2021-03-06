

import sys
sys.path.append('..')


from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

import ctypes
import pygame
import random
import signal

from Point import *
from Bridge import *

import matplotlib.pyplot as plt
import time

if sys.hexversion >= 0x03000000:
    import _thread as thread
else:
    import thread

# colors for drawing different bodies
SKELETON_COLORS = [pygame.color.THECOLORS["red"], 
                   pygame.color.THECOLORS["blue"],
                   pygame.color.THECOLORS["green"],
                   pygame.color.THECOLORS["orange"],
                   pygame.color.THECOLORS["purple"],
                   pygame.color.THECOLORS["yellow"],
                   pygame.color.THECOLORS["violet"]]


stopped = False

def signal_handler(sig, frame):
    global stopped
    print('You pressed Ctrl+C!')
    stopped = True


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
        x = [0 .. 1800]
        y = [-50 .. 1500]
left=   x -> min=27 max=957  y -> min=9   max=1439 vx -> min=-17  max=119 vy -> min=-68    max=10816 
right=  x -> min=76 max=1749 y -> min=-43 max=1440 vx -> min=-205 max=344 vy -> min=-26526 max=0
        """

        plt.ion()
        plot_fig = plt.figure()

        # self.plotters = {}
        # for i in range(6):
        # plot_ax = plot_fig.add_subplot(2,3,i + 1)

        plot_ax = plot_fig.add_subplot(1, 1, 1)

        plotter = Plotter(plot_fig, plot_ax)

        left = plotter.add_point("left", 1, colorx="r", colory="g")
        left.set_x_range(0, 2200)
        left.set_y_range(-300, 1300)
        left.set_v_range(-200000, 200000)

        right = plotter.add_point("right", 0, colorx="b", colory="y")
        right.set_x_range(0, 2200)
        right.set_y_range(-300, 1300)
        right.set_v_range(-200000, 200000)

        plotter.start_plotting()

        # self.plotters[i] = plotter
        self.plotter = plotter

        self.bridge = Bridge()
        self.data_store = None

    def store(self, t, x1, y1, x2, y2):
        if self.data_store is None:
            self.data_store = open('data.csv', 'w+')
            self.data_store.write("{}, {}, {}, {}\n".format(0, 2200,
                                                            -300, 1300))

        self.data_store.write("{}, {}, {}, {}, {}\n".format(t, x1, y1, x2, y2))


    def draw_body_bone(self, joints, jointPoints, depthPoints, color, joint0, joint1):

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

    def draw_body(self, joints, jointPoints, depthPoints, body, color):
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
        Since we have the scaling factors, perhaps we don't need the 1000 offset on y and the negative inversion
        ... this has to be checked
        """
        t = time.time()
        #t, x1, y1, _, _ = self.right.set(t, left.x, 1000 - left.y)
        #t, x2, y2, _, _ = self.left.set(t, right.x, 1000 - right.y)

        t, x1, y1, x2, y2 = self.plotter.plot(t, [left.x, right.x], [1000 - left.y, 1000 - right.y])

        # We store raw data, since ranges are saved at top of the data file
        self.store(t, x1, y1, x2, y2)


        # status = self.bridge.send_data(x1, y1, x2, y2)

        """
        # Right Leg
        self.draw_body_bone(joints, jointPoints, depthPoints, color, PyKinectV2.JointType_HipRight, PyKinectV2.JointType_KneeRight);
        self.draw_body_bone(joints, jointPoints, depthPoints, color, PyKinectV2.JointType_KneeRight, PyKinectV2.JointType_AnkleRight);
        self.draw_body_bone(joints, jointPoints, depthPoints, color, PyKinectV2.JointType_AnkleRight, PyKinectV2.JointType_FootRight);
        """

        """
        # Left Leg
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
        quit = False
        status = self.bridge.OK

        body = None

        # -------- Main Program Loop -----------
        # print("run")
        while not self._done:
            if stopped:
                break

            # --- Main event loop
            # print("run")
            for event in pygame.event.get(): # User did something
                if event.type == pygame.QUIT: # If user clicked close
                    print("stopping")
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

                    # print("body=", i)

                    joint_points = self._kinect.body_joints_to_color_space(joints)
                    depth_points = self._kinect.body_joints_to_depth_space(joints)
                    self.draw_body(joints, joint_points, depth_points, i, SKELETON_COLORS[i])
            else:
                pass

                """
                print('simulation')
                t = time.time() - self.t0
                x1 = random.random()*2000
                y1 = random.random()*2000
                x2 = random.random()*2000
                y2 = random.random()*2000

                t, x1, y1, _, _ = self.left.set(t, x1, y1)
                t, x2, y2, _, _ = self.right.set(t, x2, y2)

                self.store(t, x1, y1, x2, y2)

                self.left.scale()
                self.right.scale()

                self.left.plot()
                self.right.plot()
                time.sleep(random.random()*0.1)
                """

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

        self.data_store.close()
        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()
        pygame.quit()


__main__ = "Kinect v2 Body Game"

signal.signal(signal.SIGINT, signal_handler)
game = BodyGameRuntime();
game.run();


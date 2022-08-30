#!/usr/bin/env python

import cv2  # OpenCV
import os
import rospy
import time

from ears import Subscriber
from eyes import Video
from hands import Painter
from legs import Joystick
from mouth import Publisher

# title of the video window
WINDOW_NAME = 'Frankenstein\'s Eyes'


class Frankenstein(object):
    """
    This class provides an interface between BlueROV and ROS.
    At the moment, its main purpose is to display a video from the ROV and
    send PWM signals to the crawler wheels based on joystick trigger buttons.
    """

    ################################
    #    INITIALISATION METHODS    #
    ################################

    def __init__(self):
        self._init_successful = False

        rospy.init_node('this_is_frankenstein', log_level=rospy.DEBUG)
        rospy.on_shutdown(self.clean_up)

        if self.init_body_parts():
            rospy.loginfo("Frankenstein has risen.")
            self._init_successful = True
        else:
            rospy.signal_shutdown("Problem while starting Frankenstein, " +
                                  "shutting down...")

        # set up video recording
        self.is_recording = False
        self.video_path = os.path.expanduser('~/Videos/Frankenstein/')
        os.makedirs(self.video_path, exist_ok=True)

    def init_body_parts(self):
        if not self.init_ears():
            rospy.logerr("Could not start ears (subcriber).")
            return False
        if not self.init_mouth():
            rospy.logerr("Could not start mouth (publisher).")
            return False
        if not self.init_eyes():
            rospy.logerr("Could not start eyes (video).")
            return False
        if not self.init_legs():  # needs to be done after ears and mouth
            rospy.logerr("Could not start legs (joystick).")
            return False
        if not self.init_hands():  # needs to be done after hands
            rospy.logerr("Could not start hands (painter).")
            return False

        return True

    def init_ears(self):
        """
        Starts subscriber node to receive rostopic messages.
        """
        self.sub = Subscriber()
        return True

    def init_mouth(self):
        """
        Starts publisher node to send rostopic messages.
        """
        self.pub = Publisher()
        return True

    def init_eyes(self):
        """
        Start the camera stream to display the video.
        """
        port = rospy.get_param("/this_is_frankenstein/udp_video_port", 5610)
        rospy.loginfo("Starting Frankenstein\'s eyes on port {}".format(port))
        self.video = Video(port)
        # this would be handy to resize the window,
        # but it only works with Qt backend...
        # cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_MANUAL)
        return True

    def init_legs(self):
        """
        Start the Joystick class to handle the controller input.
        """
        self.joystick = Joystick(self.pub, self.sub)
        return True

    def init_hands(self):
        """
        Start the Painter class to communicate with painter.
        """
        self.painter = Painter(self.pub, self.sub, self.joystick)
        return True

    @property
    def init_successful(self):
        return self._init_successful

    ################################
    #       THINGS HE CAN DO       #
    ################################

    def run(self):
        """
        Frankenstein has been woken up and can do things, so he shows the
        video stream and reacts to keyboard presses.
        """
        while not rospy.is_shutdown():
            if self.video.frame_available():
                # make sure to retrieve the new frame only once
                f = self.video.frame
                cv2.imshow(WINDOW_NAME, f)
                if self.is_recording:
                    self.vw.write(f)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('s') and not self.painter.is_painting:
                self.joystick.stop_driving()

            if key == ord('f') and not self.painter.is_painting:
                self.joystick.drive_slowly_forward()

            if key == ord('b') and not self.painter.is_painting:
                self.joystick.drive_slowly_backward()

            if key == ord('p'):
                self.painter.start_painting()

            if key == ord('c'):
                self.painter.stop_painting()

            if key == ord('r'):
                if self.is_recording:
                    self.stop_recording()
                else:
                    self.start_recording()

            if key == ord('q'):
                cv2.destroyAllWindows()
                break

    def start_recording(self):
        """
        Records the current camera stream into a VideoWriter object.
        """
        self.is_recording = True
        # build filename based on current time
        t = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        self.vw = cv2.VideoWriter(
            filename=self.video_path + t + ".mp4",
            fourcc=cv2.VideoWriter_fourcc(*'mp4v'),
            fps=30.0,
            frameSize=(1920, 1080)
        )

    def stop_recording(self):
        self.is_recording = False
        self.vw.release()
        del self.vw

    ################################
    #         HOUSEKEEPING         #
    ################################

    def clean_up(self):
        """
        Automatically invoked by rospy when node is about to shut down.
        """
        rospy.loginfo("Frankenstein tired, Frankenstein sleep now.")

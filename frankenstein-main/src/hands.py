#!/usr/bin/env python
"""
This file contains the Painter class.
Usage: create a new instance of the Painter class using a publisher object
that can be used to publish the Done and Painter_Command messages
and a subscriber object that can listen to Done messages.
"""

from rospy import Duration, Timer
from legs import Joystick  # import for code auto-completion
from frankenstein_msgs.msg import Done, PainterCommand

# set for how long do we need to drive between paint strokes
DRIVING_DURATION = Duration(1)  # in seconds


class Painter(object):
    """
    Painter communication class.
    """

    def __init__(self, publisher, subscriber, joystick: Joystick):
        """
        Create Painter instance and register topics for communication.
        Args:
            publisher (ears.Publisher): a publisher object to talk.
            subscriber (mouth.Subscriber): a subscriber object to listen.
            joystick (legs.Joystick): a joystick object to control motors.
        """
        # internal status
        self.is_painting = False
        self.stop_requested = False

        # use the publisher to tell the painter what to do
        self.pub = publisher
        self.pub.register_topic("/painter_command", PainterCommand)
        self.pub.register_topic("/driving_done", Done)

        # listen to the painter
        subscriber.register_topic("/painting_done", Done, self.painting_done_callback)

        # use the joystick to drive the crawler
        self.joystick = joystick

    def start_painting(self):
        """Called from frankenstein when the user wants to start painting."""
        # early return if we're already painting
        if self.is_painting:
            return
        # we're not painting yet so let's get the ball rolling
        self.is_painting = True
        self.stop_requested = False
        # avoid accidental driving
        self.joystick.block_for_painting = True
        # tell frankenstein_painter to start
        self.pub.publish("/painter_command", PainterCommand.START_PAINTING)

    def stop_painting(self):
        """Called from frankenstein when the user wants to stop painting."""
        # early return if we're not painting anyways
        if not self.is_painting:
            return
        # we're painting and are asked to stop, so make sure the ping-pong is
        # stopped at the next iteration
        self.stop_requested = True

    def _stop_painting(self):
        """Called internally after last movement (painting to the left) is
        finished and we have been told to stop."""
        # tell painter that we want to stop painting so it can shut down
        self.pub.publish("/painter_command", PainterCommand.STOP_PAINTING)
        # make sure joystick input is re-activated
        self.joystick.block_for_painting = False
        # update internal state
        self.is_painting = False

    def painting_done_callback(self, data: Done):
        """Called when a "painting_done" message is received from the painter.

        After painting is finished, we usually drive for a bit and then let
        the painter know we're done so he can paint the next row.
        We want to stop painting if the user pressed stop, however, this
        should only happen if the linear unit is located on the left side."""

        if self.stop_requested:
            # determine if we can stop here or not
            if data.note == 'left':
                # yes, we can stop here
                self._stop_painting()
            else:
                # no, we need to paint another line
                Timer(DRIVING_DURATION, self.timer_callback, oneshot=True)
                self.joystick.drive_slowly_backward()
        else:
            # drive for a bit and then send the signal to paint again
            Timer(DRIVING_DURATION, self.timer_callback, oneshot=True)
            self.joystick.drive_slowly_backward()

    def timer_callback(self, event):
        """Called automatically by rospy.Timer after Duration is over."""
        # stop driving
        self.joystick.stop_driving()
        # tell the painter we're done with driving and he can start painting
        self.pub.publish("/driving_done", True, "driving")

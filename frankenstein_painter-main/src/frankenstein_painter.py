#!/usr/bin/env python
"""
This file contains the Painter class that provides high-level, user-friendly
access to control the D1 unit.
Usage: create a new instance of the painter, then give instructions.
The painter class has an internal state machine that keeps track of
the current state of the D1 controller.
"""

from D1_controller import D1
from frankenstein_msgs.msg import Done, PainterCommand
import rospy


class Frankenstein_Painter(object):
    """
    High-level access to D1 controller.
    Agreement: we always start painting from the left (starboard).
    """

    def __init__(self):
        rospy.init_node('this_is_painter', log_level=rospy.DEBUG)
        rospy.loginfo("Starting painter...")

        # internal status update
        self.ready_for_painting = False
        self.current_position = 'left'

        # rospy communication for driving-painting ping pong
        # listen for instructions from Frankenstein's hands
        rospy.Subscriber("/driving_done", Done, self.driving_callback)
        rospy.Subscriber("/painter_command", PainterCommand, self.command_callback)
        # tell Frankenstein's hands that we're finished
        self.pub_painting_done = rospy.Publisher("/painting_done", Done, queue_size=10)

        rospy.on_shutdown(self.clean_up)
        rospy.loginfo("Successfully started Frankenstein Painter.")

    def clean_up(self):
        """This method is automatically called by ROS during shutdown."""
        print("Painter shutting down...")

    def command_callback(self, msg: PainterCommand):
        """This method is called each time a Painter Command is received."""
        if msg.command == PainterCommand.START_PAINTING:
            # only start the controller if it is not running yet
            if not self.ready_for_painting:
                self.start_controller()
            # paint the first row
            self.paint_row()
        elif msg.command == PainterCommand.STOP_PAINTING:
            self.shut_down_controller()
        else:
            rospy.logdebug("received weird painter command %d", msg.command)

    def driving_callback(self, msg: Done):
        """We will only receive this message if driving is finished and we're
        still supposed to be painting."""
        if msg.done:
            self.paint_row()

    def paint_row(self):
        """Set a new target position for the linear unit."""

        # variables for linear unit control
        x_pos1 = 130  # first target must be away from home
        x_pos2 = 0
        x_vel = 20
        x_acc = 500
        delay = 0.1

        if self.current_position == 'left':
            rospy.logdebug("Painting from left to right...")

            # we need three strokes - left, right, left
            self.x_axis.profile_pos_mode(x_pos1, x_vel, x_acc)
            rospy.sleep(delay)
            self.x_axis.profile_pos_mode(x_pos2, x_vel, x_acc)
            rospy.sleep(delay)
            self.x_axis.profile_pos_mode(x_pos1, x_vel, x_acc)
            rospy.sleep(delay)

            # update current position so we know for the next row
            self.current_position = 'right'

        elif self.current_position == 'right':
            rospy.logdebug("Painting from right to left...")

            # we need three strokes - right, left, right
            self.x_axis.profile_pos_mode(x_pos2, x_vel, x_acc)
            rospy.sleep(delay)
            self.x_axis.profile_pos_mode(x_pos1, x_vel, x_acc)
            rospy.sleep(delay)
            self.x_axis.profile_pos_mode(x_pos2, x_vel, x_acc)
            rospy.sleep(delay)

            # update current position so we know for the next row
            self.current_position = 'left'

        # report back that we're done and where we are
        self.pub_painting_done.publish(True, self.current_position)

    def start_controller(self):
        """Initialise D1 controller and home them before first use."""
        rospy.logdebug("Starting controller...")

        # start and home controller
        self.x_axis = D1("169.254.98.9", 502, "Linear X-Axis", False)
        self.x_axis.homing(60, 600)

        # update our internal state
        self.ready_for_painting = True

    def shut_down_controller(self):
        rospy.logdebug("Shutting down controller...")

        # update our internal state
        self.ready_for_painting = False

        # shut down controller
        self.x_axis.set_shutdown()


if __name__ == '__main__':
    fp = Frankenstein_Painter()
    rospy.spin()

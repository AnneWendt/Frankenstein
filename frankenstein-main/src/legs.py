#!/usr/bin/env python
"""
This file contains the Joystick class. It can read input from a joystick
controller and convert it to a format that Frankenstein can use.
Usage: create a new instance of the Joystick class using a publisher object
that can be used to publish the Motor messages and a subscriber object that
can listen to joystick messages.
The Joystick class then builds and publishes Motor messages.
"""

import rospy
from sensor_msgs.msg import Joy
from frankenstein_msgs.msg import Motor

# TODO: find a good speed for driving slowly while painting
SLOW_SPEED = 20


class Joystick(object):
    """
    Joystick handling class.
    """

    def __init__(self, publisher, subscriber):
        """
        Create Joystick instance and register topics for communication.
        Args:
            publisher (ears.Publisher): a publisher object.
            subscriber (mouth.Subscriber): a subscriber object.
        """
        self.pub = publisher
        self.pub.register_topic("/motor", Motor)
        subscriber.register_topic("/joy", Joy, self.joystick_callback)
        self.block_for_painting = False

    def joystick_callback(self, data):
        """
        Convert the joystick commands to Motor message format.
        This method is automatically called through the Subscriber class
        each time a joystick message is received.
        This has only been tested with a wired XBox controller for now.
        The /joystick data may have a different format for other controllers.
        """
        if self.block_for_painting:
            # don't react to joystick input while we're painting
            return

        # this part is definitely specific to the controller configuration
        # axes are float, buttons are int (where 0 = released and 1 = pressed)
        # the array indices were retrieved using "jstest /dev/input/js2"
        left_trigger = data.axes[2]
        right_trigger = data.axes[5]
        left_bumper = data.buttons[4] == 1
        right_bumper = data.buttons[5] == 1

        rospy.logdebug("left trigger  %f", left_trigger)
        rospy.logdebug("right trigger %f", right_trigger)
        rospy.logdebug("left bumper  %s", left_bumper)
        rospy.logdebug("right bumper %s", right_bumper)

        # handling the input COULD BE specific to the controller configuration
        # e.g., value ranges or default values could be different

        # for our XBox controller, the following applies:
        # the trigger values range from -1.0 to +1.0
        # -1.0 means the trigger is fully pressed
        # 0.0 had an ambiguity but this was resolved by setting the
        # joystick parameter default_trig_val to true
        # +1.0 means that the trigger is fully released at the moment

        # trigger value is between -1 (high) and +1 (low)
        # target value is duty cycle in percent with 0 (low) and 100 (high)
        # Step 1: turn trigger around to -1 (low) and +1 (high)
        # target = -1 * trigger
        # Step 2: move starting point to 0 (low) and 2 (high)
        # target = (-1 * trigger) + 1
        # Step 3: scale up to 0 (low) and 100 (high)
        # target = ((-1 * trigger) + 1) * 50
        # Step 4: simplification
        # target = (1 - trigger) * 50

        # ignore 1 because we will enter this code if *any* button is pressed
        # and we don't want to send messages unless we actually have values
        if left_trigger < 1:
            mode = Motor.LEFT_BACKWARD if left_bumper else Motor.LEFT_FORWARD
            dc = int(50 * (1 - left_trigger))
            self._set_motor(mode, dc)

        if right_trigger < 1:
            mode = Motor.RIGHT_BACKWARD if right_bumper else Motor.RIGHT_FORWARD
            dc = int(50 * (1 - right_trigger))
            self._set_motor(mode, dc)

    def _set_motor(self, mode, dc, force_dc=False):
        """
        Sets a motor to a PWM value by publishing to /motor topic. It will cut
        off very small dc values (< 20) because sometimes the joystick node
        can not keep up if the triggers are released too quickly, which could
        lead to very slow forward movement even if the user wanted to stop.
        Args:
            mode (uint8): driving mode from frankenstein_msgs.Motors class.
            dc (int16): duty cycle ranging from 0 to 100.
            force_dc (bool, optional): do *not* cut off very small dc value.
        """
        if not force_dc:
            if dc < 20:
                dc = 0

        self.pub.publish("/motor", mode, dc)

    def drive_slowly_forward(self):
        self._set_motor(mode=Motor.LEFT_FORWARD, dc=SLOW_SPEED, force_dc=True)
        self._set_motor(mode=Motor.RIGHT_FORWARD, dc=SLOW_SPEED, force_dc=True)

    def drive_slowly_backward(self):
        self._set_motor(mode=Motor.LEFT_BACKWARD, dc=SLOW_SPEED, force_dc=True)
        self._set_motor(mode=Motor.RIGHT_BACKWARD, dc=SLOW_SPEED, force_dc=True)

    def stop_driving(self):
        self._set_motor(mode=Motor.LEFT_FORWARD, dc=0)
        self._set_motor(mode=Motor.RIGHT_FORWARD, dc=0)

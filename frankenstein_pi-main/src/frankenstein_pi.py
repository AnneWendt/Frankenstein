#!/usr/bin/env python

import pigpio
import rospy
from frankenstein_msgs.msg import Motor


# possible GPIO PWM pins that also work for hardware PWM:
#  BOARD |  BCM
# -------+--------
#    12  | GPIO18
#    32  | GPIO12
#    33  | GPIO13
#    35  | GPIO19


LEFT_DC_PIN = 12  # PIN 32
RIGHT_DC_PIN = 13  # PIN 33
LEFT_DIR_PIN = 16  # PIN 36
RIGHT_DIR_PIN = 26  # PIN 37
FREQUENCY = 10000  # tested for motor frequency


class Frankenstein_Pi(object):

    def __init__(self):
        rospy.init_node('this_is_pi', log_level=rospy.DEBUG)
        rospy.loginfo("Starting Frankenstein on Pi...")
        self.ok = False

        self.pi = pigpio.pi()
        if not self.pi.connected:
            rospy.logerr("Pigpio could not connect to pins!")
            rospy.signal_shutdown("Initialisation did not complete properly.")
            # avoid testing pins, starting Subscriber and setting on_shutdown
            return

        # verify pins work for hardware PWM
        # pigpio return code will be negative if an error occured
        # http://abyz.me.uk/rpi/pigpio/python.html#errnum
        ret = self.pi.hardware_PWM(LEFT_DC_PIN, FREQUENCY, 0)
        if ret < 0:
            rospy.logerr('pigpio error for GPIO pin %d:\n%s'
                         % (LEFT_DC_PIN, pigpio.error_text(ret)))
        ret = self.pi.hardware_PWM(RIGHT_DC_PIN, FREQUENCY, 0)
        if ret < 0:
            rospy.logerr('pigpio error for GPIO pin %d:\n%s'
                         % (RIGHT_DC_PIN, pigpio.error_text(ret)))

        # initialise direction pins
        self.pi.set_mode(LEFT_DIR_PIN, pigpio.OUTPUT)
        self.pi.set_mode(RIGHT_DIR_PIN, pigpio.OUTPUT)
        self.pi.write(LEFT_DIR_PIN, pigpio.LOW)
        self.pi.write(RIGHT_DIR_PIN, pigpio.HIGH)  # sic!

        # rospy node setup stuff
        rospy.Subscriber("/motor", Motor, self.callback)
        rospy.on_shutdown(self.clean_up)
        self.ok = True
        rospy.loginfo("Successfully started Frankenstein on Pi.")

    def callback(self, msg):
        rospy.logdebug("dc value: %d", msg.dc)
        if msg.dc < 0 or msg.dc > 100:
            rospy.logwarn("dc value out of range [0, 100]")
            return

        # Faulhaber motor "direction" PIN:
        # turn left = pigpio.LOW
        # turn right = pigpio.HIGH
        # BUT! level converter probably negates these

        if msg.mode == Motor.LEFT_FORWARD:
            rospy.logdebug("LEFT FWD value %d", msg.dc)
            self.pi.hardware_PWM(LEFT_DC_PIN, FREQUENCY, msg.dc * 10000)
            self.pi.write(LEFT_DIR_PIN, pigpio.LOW)
        elif msg.mode == Motor.LEFT_BACKWARD:
            rospy.logdebug("LEFT BWD value %d", msg.dc)
            self.pi.hardware_PWM(LEFT_DC_PIN, FREQUENCY, msg.dc * 10000)
            self.pi.write(LEFT_DIR_PIN, pigpio.HIGH)
        elif msg.mode == Motor.RIGHT_FORWARD:
            rospy.logdebug("RIGHT FWD value %d", msg.dc)
            self.pi.hardware_PWM(RIGHT_DC_PIN, FREQUENCY, msg.dc * 10000)
            self.pi.write(RIGHT_DIR_PIN, pigpio.HIGH)
        elif msg.mode == Motor.RIGHT_BACKWARD:
            rospy.logdebug("RIGHT BWD value %d", msg.dc)
            self.pi.hardware_PWM(RIGHT_DC_PIN, FREQUENCY, msg.dc * 10000)
            self.pi.write(RIGHT_DIR_PIN, pigpio.LOW)
        else:
            rospy.logdebug("received weird motor mode %d", msg.mode)

    def clean_up(self):
        self.pi.hardware_PWM(LEFT_DC_PIN, FREQUENCY, 0)
        self.pi.hardware_PWM(RIGHT_DC_PIN, FREQUENCY, 0)
        self.pi.write(LEFT_DIR_PIN, pigpio.LOW)
        self.pi.write(RIGHT_DIR_PIN, pigpio.LOW)
        self.pi.stop()

    def run(self):
        self.pi.hardware_PWM(LEFT_DC_PIN, FREQUENCY, 0 * 10000)
        self.pi.hardware_PWM(RIGHT_DC_PIN, FREQUENCY, 0 * 10000)
        rospy.spin()


if __name__ == '__main__':
    fpi = Frankenstein_Pi()
    if fpi.ok:
        fpi.run()

#!/usr/bin/env python
"""
This file contains the Publisher class that can publish rostopic messages.
Usage: create a new instance of the Publisher class,
register a topic and publish through it.
"""

import rospy


class Publisher(object):
    """
    Topic publishing class.
    """

    def __init__(self):
        """
        Create a container that can hold all the topic publisher handles.
        """
        self.pub_dict = {}

    def register_topic(self, topic: str, msg_type) -> None:
        """
        Adds a topic to our pub handle dict and
        automatically creates the ROS publisher object.
        Args:
            topic (str): the full topic name.
            msg_type (Message Class): the type of message that will be sent.
        """
        if topic not in self.pub_dict:
            self.pub_dict[topic] = rospy.Publisher(topic, msg_type, queue_size=10)

    def publish(self, topic: str, *args, **kwargs) -> None:
        """
        Sends off the message via ROS central message broker.
        Args:
            topic (str): the full topic name.
            args/kwargs (optional): the message data/content.
        """
        if topic not in self.pub_dict:
            rospy.logerr("Topic %s has not been registered with publisher yet!", topic)
            return

        rospy.logdebug("Publishing topic " + topic +
                       " with args " + str(args) +
                       " and kwargs " + str(kwargs))
        self.pub_dict[topic].publish(*args, **kwargs)

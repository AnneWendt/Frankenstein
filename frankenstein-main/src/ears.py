#!/usr/bin/env python
"""
This file contains the Subscriber class that can subscribe to rostopics.
Usage: create a new instance of the Subscriber class,
register a topic and you will get notifications.
Unregister and you will stop getting notifications.
Design decision: Each topic can only be connected to one callback function.
"""

import rospy


class Subscriber(object):
    """
    Topic subscription class.
    """
    def __init__(self):
        """
        Create a container that can hold all the topic susbcriber handles.
        """
        self.sub_dict = {}

    def register_topic(self, topic: str, msg_type, callback) -> None:
        """
        We allow only one subscriber per topic. This will hopefully keep the
        main code a bit cleaner and avoid accidental double-subscriptions.
        Args:
            topic (str): the full topic name.
            msg_type (Message Class): the message type that will be received.
            callback (function(data)): the function that will be called when
                                       a message is received.
        """
        if topic not in self.sub_dict:
            self.sub_dict[topic] = rospy.Subscriber(topic, msg_type, callback)

    def unregister_topic(self, topic: str) -> None:
        """
        Stop listening to a topic and remove it from our private container.
        Args:
            topic (str): the full topic name.
        """
        sub = self.sub_dict.pop(topic, None)
        # in case we didn't actually have this topic in our list,
        # dict.pop() would default to None, so avoid NullPointer
        if sub is not None:
            sub.unregister()

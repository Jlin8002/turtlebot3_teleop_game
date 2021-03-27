from dataclasses import dataclass
import rospy

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist


QUEUE_SIZE = 10


class Sensor:
    def __init__(self, publishers, subscribers):
        rospy.init_node("teleop_game")
        self.initialize_publishers(publishers)
        self.initialize_subscribers(subscribers)

    def initialize_publishers(self, publishers):
        self.publishers = {
            topic_name: rospy.Publisher(topic_name, datatype, queue_size=QUEUE_SIZE)
            for (topic_name, datatype) in publishers
        }

    def initialize_subscribers(self, subscribers):
        self.subscribers = {
            topic_name: rospy.Subscriber(
                topic_name, datatype, self.handle_msg, topic_name
            )
            for (topic_name, datatype) in subscribers
        }
        self.data = {topic_name: None for (topic_name, _) in subscribers}

    def handle_msg(self, msg, topic_name):
        self.data[topic_name] = msg

    def publish(self, *args):
        for (key, value) in args:
            try:
                print(f"Publishing {value}")
                self.publishers[key].publish(value)
            except KeyError:
                print(f"No publisher found with topic name {key}.")

    def get_msg(self, topic_name):
        try:
            return self.data[topic_name]
        except KeyError:
            print(f"No subscriber found with topic name {topic_name}.")
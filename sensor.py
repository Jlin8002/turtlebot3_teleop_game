import rospy

from typing import Dict, List, Optional, Tuple

from msg import ROSMsg, MsgType, default_value

QUEUE_SIZE = 10


class Sensor:
    """
    An interface for storing the latest subscriber values and publishing to topics.
    """

    def __init__(self, publishers: List[ROSMsg], subscribers: List[ROSMsg]) -> None:
        rospy.init_node("teleop_game")
        self.initialize_publishers(publishers)
        self.initialize_subscribers(subscribers)

    def initialize_publishers(self, publishers: List[ROSMsg]) -> None:
        """
        Set up publishers for the specified topics and types.
        """
        self.publishers = {
            ros_msg.topic: rospy.Publisher(
                ros_msg.topic, ros_msg.type, queue_size=QUEUE_SIZE
            )
            for ros_msg in publishers
        }

    def initialize_subscribers(self, subscribers: List[ROSMsg]) -> None:
        """
        Set up subscribers for the specified topics and types.
        """
        self.subscribers = {
            ros_msg.topic: rospy.Subscriber(
                ros_msg.topic, ros_msg.type, self.handle_msg, ros_msg.topic
            )
            for ros_msg in subscribers
        }
        self.data: Dict[str, Optional[MsgType]] = {
            msg.topic: None for msg in subscribers
        }

    def handle_msg(self, msg: MsgType, topic: str) -> None:
        """
        Store the latest data from a subscriber.
        """
        self.data[topic] = msg

    def publish(self, ros_msg: ROSMsg, msg: MsgType) -> None:
        """
        Publish a message to a topic.
        """
        try:
            self.publishers[ros_msg.topic].publish(msg)
        except KeyError:
            print(f"No publisher found with topic name {ros_msg.topic}.")

    def get_msg(self, ros_msg: ROSMsg) -> Optional[MsgType]:
        """
        Get the latest message from a topic.
        If no messages have been received, return a default message.
        """
        try:
            if ((latest_msg := self.data[ros_msg.topic]) is not None) and isinstance(
                latest_msg, ros_msg.type
            ):
                return latest_msg
            return default_value(ros_msg)
        except KeyError:
            print(f"No subscriber found with topic name {ros_msg.topic}.")
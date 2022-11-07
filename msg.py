from typing import Union

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan


class ROSMsg:
    """
    Label a ROS message by the topic it originated from and the type of data it holds.
    """

    def __init__(self, topic: str, datatype: type):
        self.topic = topic
        self.type = datatype

    topic: str
    datatype: type


MsgType = Union[Twist, Odometry, Image, LaserScan]


def default_value(msg: ROSMsg) -> MsgType:
    """
    Return a ROS message initialized with no arguments.
    """
    return msg.type()

class TeleopMsgs:
    def __init__(self, cmd_vel="cmd_vel", image_raw="camera/rgb/image_raw", odom="odom", scan="scan"): 
        self.cmd_vel = ROSMsg(topic=cmd_vel, datatype=Twist)
        self.image_raw = ROSMsg(topic=image_raw, datatype=Image)
        self.odom = ROSMsg(topic=odom, datatype=Odometry)
        self.scan = ROSMsg(topic=scan, datatype=LaserScan)
        self.publisher_msgs = [self.cmd_vel]
        self.subscriber_msgs = [self.cmd_vel, self.image_raw, self.odom, self.scan]
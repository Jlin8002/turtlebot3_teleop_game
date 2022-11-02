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

CMD_VEL_MSG = ROSMsg(topic="cmd_vel", datatype=Twist)
IMAGE_MSG = ROSMsg(topic="camera/rgb/image_raw", datatype=Image)
ODOM_MSG = ROSMsg(topic="odom", datatype=Odometry)
SCAN_MSG = ROSMsg(topic="scan", datatype=LaserScan)


def default_value(msg: ROSMsg) -> MsgType:
    """
    Return a ROS message initialized with no arguments.
    """
    return msg.type()
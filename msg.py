from dataclasses import dataclass
from typing import Union

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan


@dataclass
class ROSMsg:
    topic: str
    type: type


MsgType = Union[Twist, Odometry, Image, LaserScan]

CMD_VEL_MSG = ROSMsg(topic="cmd_vel", type=Twist)
IMAGE_MSG = ROSMsg(topic="camera/rgb/image_raw", type=Image)
ODOM_MSG = ROSMsg(topic="odom", type=Odometry)
SCAN_MSG = ROSMsg(topic="scan", type=LaserScan)


def default_value(msg: ROSMsg) -> MsgType:
    return msg.type()
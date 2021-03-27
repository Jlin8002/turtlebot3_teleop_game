#!/usr/bin/env python3

from sensor import Sensor
import numpy as np
import pygame
import rospy

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan

CMD_VEL_TOPIC = "cmd_vel"
IMAGE_TOPIC = "camera/rgb/image_raw"
ODOM_TOPIC = "odom"
SCAN_TOPIC = "scan"

CMD_VEL_MSG = (CMD_VEL_TOPIC, Twist)
IMAGE_MSG = (IMAGE_TOPIC, Image)
ODOM_MSG = (ODOM_TOPIC, Odometry)
SCAN_MSG = (SCAN_TOPIC, LaserScan)

PUBLISHER_DATATYPES = [CMD_VEL_MSG]
SUBSCRIBER_DATATYPES = [CMD_VEL_MSG, IMAGE_MSG, ODOM_MSG, SCAN_MSG]


WIDTH, HEIGHT = 900, 500

WHITE = (255, 255, 255)

FPS = 10

ACC_LIN = 0.1  # meters per second
ACC_ANG = 0.1  # meters per second
DEC_LIN = 0.05
DEC_ANG = 0.2
MAX_LIN = 1
MIN_LIN = -0.5
MAX_ANG = 2
MIN_ANG = -2
TURN_FACTOR = 1.5

KEY_MOVE_FORWARD = pygame.K_w
KEY_MOVE_BACKWARD = pygame.K_s
KEY_TURN_LEFT = pygame.K_a
KEY_TURN_RIGHT = pygame.K_d
KEY_ROTATE_LEFT = pygame.K_q
KEY_ROTATE_RIGHT = pygame.K_e
KEY_STOP = pygame.K_SPACE


class UI:
    def __init__(self):
        self.bridge = CvBridge()
        self.sensors = Sensor(PUBLISHER_DATATYPES, SUBSCRIBER_DATATYPES)
        pygame.init()
        self.window = pygame.display.set_mode((WIDTH, HEIGHT))
        self.window.fill(WHITE)
        self.keys_pressed = pygame.key.get_pressed()

    def get_image(self):
        if (image := self.sensors.get_msg(IMAGE_TOPIC)) is not None:
            return self.bridge.imgmsg_to_cv2(image)
        return None

    def get_twist(self):
        return self.sensors.get_msg(CMD_VEL_TOPIC)

    def get_pose(self):
        if (odom := self.sensors.get_msg(ODOM_MSG)) :
            return odom.pose.pose
        return None

    def send_control_input(self):
        if not (current_twist := self.get_twist()):
            current_twist = Twist()

        (vel_lin, vel_ang) = (
            gas(current_twist.linear.x, self.keys_pressed),
            steer(current_twist.linear.x, self.keys_pressed)
            + rotate(current_twist.angular.z, self.keys_pressed),
        )
        new_twist = Twist()
        new_twist.linear.x = vel_lin
        new_twist.angular.z = vel_ang
        self.sensors.publish((CMD_VEL_TOPIC, new_twist))

    def set_background(self):
        if (image := self.get_image()) is not None:
            pygame_image = pygame.image.frombuffer(
                image, (image.shape[1], image.shape[0]), "RGB"
            )
            pygame_image_resized = pygame.transform.scale(pygame_image, (WIDTH, HEIGHT))
            self.window.blit(pygame_image_resized, (0, 0))

    def draw_window(self):
        pygame.display.update()

    def update(self):
        self.keys_pressed = pygame.key.get_pressed()
        self.send_control_input()
        self.set_background()
        self.draw_window()


def gas(lin_vel, keys_pressed):
    if keys_pressed[KEY_STOP]:
        return 0

    if keys_pressed[KEY_MOVE_FORWARD] == keys_pressed[KEY_MOVE_BACKWARD]:
        if lin_vel > DEC_LIN:
            return max(lin_vel - DEC_LIN, 0)
        if lin_vel < -DEC_LIN:
            return min(lin_vel + DEC_LIN, 0)
        return 0

    if keys_pressed[KEY_MOVE_FORWARD]:
        return min(lin_vel + ACC_LIN, MAX_LIN)
    if keys_pressed[KEY_MOVE_BACKWARD]:
        return max(lin_vel - ACC_LIN, MIN_LIN)

    return lin_vel


def steer(lin_vel, keys_pressed):
    if keys_pressed[KEY_STOP]:
        return 0

    direction = -1 if lin_vel < 0 else 1

    if keys_pressed[KEY_TURN_LEFT] == keys_pressed[KEY_TURN_RIGHT]:
        return 0
    if keys_pressed[KEY_TURN_LEFT]:
        return min(abs(lin_vel) * TURN_FACTOR, MAX_ANG) * direction
    if keys_pressed[KEY_TURN_RIGHT]:
        return max(abs(lin_vel) * -TURN_FACTOR, MIN_ANG) * direction

    return lin_vel


def rotate(ang_vel, keys_pressed):
    if keys_pressed[KEY_STOP]:
        return 0

    if keys_pressed[KEY_ROTATE_LEFT] == keys_pressed[KEY_ROTATE_RIGHT]:
        return 0

    if keys_pressed[KEY_ROTATE_LEFT]:
        return min(ang_vel + ACC_ANG, MAX_ANG)
    if keys_pressed[KEY_ROTATE_RIGHT]:
        return max(ang_vel - ACC_ANG, MIN_ANG)

    return ang_vel


def run():
    clock = pygame.time.Clock()
    run = True
    ui = UI()

    while run:
        clock.tick(FPS)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

        ui.update()

    pygame.quit()


if __name__ == "__main__":
    run()
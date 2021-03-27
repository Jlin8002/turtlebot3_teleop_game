#!/usr/bin/env python3

from sensor import Sensor
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


pygame.init()
WIDTH, HEIGHT = 900, 500
WIN = pygame.display.set_mode((WIDTH, HEIGHT))

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


def draw_window():
    WIN.fill(WHITE)
    pygame.display.update()


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
    sensors = Sensor(PUBLISHER_DATATYPES, SUBSCRIBER_DATATYPES)
    while run:
        clock.tick(FPS)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

        if not (current_twist := sensors.get_msg(CMD_VEL_TOPIC)):
            current_twist = Twist()
        current_odom = sensors.get_msg(ODOM_TOPIC)
        current_scan = sensors.get_msg(SCAN_TOPIC)
        current_image = sensors.get_msg(IMAGE_TOPIC)

        keys_pressed = pygame.key.get_pressed()
        (vel_lin, vel_ang) = (
            gas(current_twist.linear.x, keys_pressed),
            steer(current_twist.linear.x, keys_pressed)
            + rotate(current_twist.angular.z, keys_pressed),
        )
        new_twist = Twist()
        new_twist.linear.x = vel_lin
        new_twist.angular.z = vel_ang
        sensors.publish((CMD_VEL_TOPIC, new_twist))

        draw_window()

    pygame.quit()


if __name__ == "__main__":
    run()
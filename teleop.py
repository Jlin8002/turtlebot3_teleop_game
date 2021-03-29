import pygame

from typing import List

from geometry_msgs.msg import Twist

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


def gas(lin_vel: float, keys_pressed: List[bool]) -> float:
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


def steer(lin_vel: float, keys_pressed: List[bool]) -> float:
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


def rotate(ang_vel: float, keys_pressed: List[bool]) -> float:
    if keys_pressed[KEY_STOP]:
        return 0

    if keys_pressed[KEY_ROTATE_LEFT] == keys_pressed[KEY_ROTATE_RIGHT]:
        return 0

    if keys_pressed[KEY_ROTATE_LEFT]:
        return min(ang_vel + ACC_ANG, MAX_ANG)
    if keys_pressed[KEY_ROTATE_RIGHT]:
        return max(ang_vel - ACC_ANG, MIN_ANG)

    return ang_vel


def calculate_teleop_twist(current_twist, keys_pressed: List[bool]) -> Twist:
    (vel_lin, vel_ang) = (
        gas(current_twist.linear.x, keys_pressed),
        steer(current_twist.linear.x, keys_pressed)
        + rotate(current_twist.angular.z, keys_pressed),
    )
    new_twist = Twist()
    new_twist.linear.x = vel_lin
    new_twist.angular.z = vel_ang

    return new_twist

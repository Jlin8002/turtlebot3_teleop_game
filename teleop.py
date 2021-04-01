import pygame

from typing import List

from geometry_msgs.msg import Twist

ACC_LIN = 0.01  # meters per second
ACC_ANG = 0.01  # meters per second
DEC_LIN = 0.005
DEC_ANG = 0.005
MAX_LIN = 1
MIN_LIN = -0.5
MAX_ANG = 2
MIN_ANG = -2
TURN_FACTOR = 2

KEY_MOVE_FORWARD = pygame.K_w
KEY_MOVE_BACKWARD = pygame.K_s
KEY_TURN_LEFT = pygame.K_a
KEY_TURN_RIGHT = pygame.K_d
KEY_ROTATE_LEFT = pygame.K_q
KEY_ROTATE_RIGHT = pygame.K_e
KEY_STOP = pygame.K_SPACE


def bound(x: float, lower: float, upper: float) -> float:
    """
    Restrict a number to a range between a lower and upper bound.
    """
    return min(max(x, lower), upper)


def gas(lin_vel: float, keys_pressed: List[bool]) -> float:
    """
    Calculate a linear velocity for the robot based on keyboard input.
    """
    if keys_pressed[KEY_STOP]:
        return 0

    if keys_pressed[KEY_MOVE_FORWARD] == keys_pressed[KEY_MOVE_BACKWARD]:
        if lin_vel > DEC_LIN:
            return max(lin_vel - DEC_LIN, 0)
        if lin_vel < -DEC_LIN:
            return min(lin_vel + DEC_LIN, 0)
        return 0

    if keys_pressed[KEY_MOVE_FORWARD]:
        return bound(lin_vel + ACC_LIN, MIN_LIN, MAX_LIN)
    if keys_pressed[KEY_MOVE_BACKWARD]:
        return bound(lin_vel - ACC_LIN, MIN_LIN, MAX_LIN)

    return max(min(lin_vel, MAX_LIN), MIN_LIN)


def steer(lin_vel: float, keys_pressed: List[bool]) -> float:
    """
    Calculate an angular velocity for the robot based on keyboard input
    using steering mechanics (turnable wheels).
    """
    if keys_pressed[KEY_STOP]:
        return 0

    direction = -1 if lin_vel < 0 else 1

    if keys_pressed[KEY_TURN_LEFT] == keys_pressed[KEY_TURN_RIGHT]:
        return 0
    if keys_pressed[KEY_TURN_LEFT]:
        return bound(abs(lin_vel) * TURN_FACTOR, MIN_ANG, MAX_ANG) * direction
    if keys_pressed[KEY_TURN_RIGHT]:
        return bound(abs(lin_vel) * -TURN_FACTOR, MIN_ANG, MAX_ANG) * direction

    return 0


def pivot(ang_vel: float, keys_pressed: List[bool]) -> float:
    """
    Calculate an angular velocity for the robot based on keyboard input
    using pivot mechanics (wheels moving in opposite directions).
    """
    if keys_pressed[KEY_STOP]:
        return 0

    if keys_pressed[KEY_ROTATE_LEFT] == keys_pressed[KEY_ROTATE_RIGHT]:
        if (
            keys_pressed[KEY_TURN_LEFT]
            or keys_pressed[KEY_TURN_RIGHT]
            or keys_pressed[KEY_MOVE_FORWARD]
            or keys_pressed[KEY_MOVE_BACKWARD]
        ):
            return 0
        if ang_vel > DEC_ANG:
            return max(ang_vel - DEC_ANG, 0)
        if ang_vel < -DEC_ANG:
            return min(ang_vel + DEC_ANG, 0)
        return 0

    if keys_pressed[KEY_ROTATE_LEFT]:
        return bound(ang_vel + ACC_ANG, MIN_ANG, MAX_ANG)
    if keys_pressed[KEY_ROTATE_RIGHT]:
        return bound(ang_vel - ACC_ANG, MIN_ANG, MAX_ANG)

    return bound(ang_vel, MIN_ANG, MAX_ANG)


def calculate_teleop_twist(current_twist, keys_pressed: List[bool]) -> Twist:
    """
    Get a new twist for the robot based on keyboard input.
    """
    (vel_lin, vel_ang) = (
        gas(current_twist.linear.x, keys_pressed),
        steer(current_twist.linear.x, keys_pressed)
        + pivot(current_twist.angular.z, keys_pressed),
    )
    new_twist = Twist()
    new_twist.linear.x = vel_lin
    new_twist.angular.z = vel_ang

    return new_twist

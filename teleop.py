from typing import List

from config import Config

from geometry_msgs.msg import Twist



def bound(x: float, lower: float, upper: float) -> float:
    """
    Restrict a number to a range between a lower and upper bound.
    """
    return min(max(x, lower), upper)


def gas(lin_vel: float, config: Config, keys_pressed: List[bool]) -> float:
    """
    Calculate a linear velocity for the robot based on keyboard input.
    """
    if keys_pressed[config.KEY_STOP]:
        return 0

    if keys_pressed[config.KEY_MOVE_FORWARD] == keys_pressed[config.KEY_MOVE_BACKWARD]:
        if lin_vel > config.DEC_LIN:
            return max(lin_vel - config.DEC_LIN, 0)
        if lin_vel < -config.DEC_LIN:
            return min(lin_vel + config.DEC_LIN, 0)
        return 0

    if keys_pressed[config.KEY_MOVE_FORWARD]:
        return bound(lin_vel + config.ACC_LIN, config.MIN_LIN, config.MAX_LIN)
    if keys_pressed[config.KEY_MOVE_BACKWARD]:
        return bound(lin_vel - config.ACC_LIN, config.MIN_LIN, config.MAX_LIN)

    return max(min(lin_vel, config.MAX_LIN), config.MIN_LIN)


def steer(lin_vel: float, config: Config, keys_pressed: List[bool]) -> float:
    """
    Calculate an angular velocity for the robot based on keyboard input
    using steering mechanics (turnable wheels).
    """
    if keys_pressed[config.KEY_STOP]:
        return 0

    direction = -1 if lin_vel < 0 else 1

    if keys_pressed[config.KEY_TURN_LEFT] == keys_pressed[config.KEY_TURN_RIGHT]:
        return 0
    if keys_pressed[config.KEY_TURN_LEFT]:
        return bound(abs(lin_vel) * config.TURN_FACTOR, config.MIN_ANG, config.MAX_ANG) * direction
    if keys_pressed[config.KEY_TURN_RIGHT]:
        return bound(abs(lin_vel) * -config.TURN_FACTOR, config.MIN_ANG, config.MAX_ANG) * direction

    return 0


def pivot(ang_vel: float, config: Config, keys_pressed: List[bool]) -> float:
    """
    Calculate an angular velocity for the robot based on keyboard input
    using pivot mechanics (wheels moving in opposite directions).
    """
    if keys_pressed[config.KEY_STOP]:
        return 0

    if keys_pressed[config.KEY_ROTATE_LEFT] == keys_pressed[config.KEY_ROTATE_RIGHT]:
        if (
            keys_pressed[config.KEY_TURN_LEFT]
            or keys_pressed[config.KEY_TURN_RIGHT]
            or keys_pressed[config.KEY_MOVE_FORWARD]
            or keys_pressed[config.KEY_MOVE_BACKWARD]
        ):
            return 0
        if ang_vel > config.DEC_ANG:
            return max(ang_vel - config.DEC_ANG, 0)
        if ang_vel < -config.DEC_ANG:
            return min(ang_vel + config.DEC_ANG, 0)
        return 0

    if keys_pressed[config.KEY_ROTATE_LEFT]:
        return bound(ang_vel + config.ACC_ANG, config.MIN_ANG, config.MAX_ANG)
    if keys_pressed[config.KEY_ROTATE_RIGHT]:
        return bound(ang_vel - config.ACC_ANG, config.MIN_ANG, config.MAX_ANG)

    return bound(ang_vel, config.MIN_ANG, config.MAX_ANG)


def calculate_teleop_twist(current_twist: Twist, config: Config, keys_pressed: List[bool]) -> Twist:
    """
    Get a new twist for the robot based on keyboard input.
    """
    (vel_lin, vel_ang) = (
        gas(current_twist.linear.x, config, keys_pressed),
        steer(current_twist.linear.x, config, keys_pressed)
        + pivot(current_twist.angular.z, config, keys_pressed),
    )
    new_twist = Twist()
    new_twist.linear.x = vel_lin
    new_twist.angular.z = vel_ang

    return new_twist

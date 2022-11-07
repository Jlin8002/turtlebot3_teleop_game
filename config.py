import json
import pygame

from typing import Optional, Union

class Config:

    def __init__(self, config_robot: dict, config_keys: dict):
        self.CMD_VEL = check_exist_str(config_robot, "topics/cmd_vel", "cmd_vel")
        self.IMAGE_RAW = check_exist_str(config_robot, "topics/image_raw", "camera/rgb/image_raw")
        self.ODOM = check_exist_str(config_robot, "topics/odom", "odom")
        self.SCAN = check_exist_str(config_robot, "topics/scan", "scan")

        self.ACC_LIN = check_exist_float(config_robot, "constants/acc_lin", 0.01)  # meters per second
        self.ACC_ANG = check_exist_float(config_robot, "constants/acc_ang", 0.01)  # meters per second
        self.DEC_LIN = check_exist_float(config_robot, "constants/dec_lin", 0.005)
        self.DEC_ANG = check_exist_float(config_robot, "constants/dec_ang", 0.005)
        self.MAX_LIN = check_exist_float(config_robot, "constants/max_lin", 1)
        self.MIN_LIN = check_exist_float(config_robot, "constants/min_lin", -0.5)
        self.MAX_ANG = check_exist_float(config_robot, "constants/max_ang", 2)
        self.MIN_ANG = check_exist_float(config_robot, "constants/min_ang", -2)
        self.TURN_FACTOR = check_exist_float(config_robot, "constants/turn_factor", 2)

        self.KEY_MOVE_FORWARD = check_exist_key(config_keys, "forward", pygame.K_w)
        self.KEY_MOVE_BACKWARD = check_exist_key(config_keys, "backward", pygame.K_s)
        self.KEY_TURN_LEFT = check_exist_key(config_keys, "turn_left", pygame.K_a)
        self.KEY_TURN_RIGHT = check_exist_key(config_keys, "turn_right", pygame.K_d)
        self.KEY_ROTATE_LEFT = check_exist_key(config_keys, "rotate_left", pygame.K_q)
        self.KEY_ROTATE_RIGHT = check_exist_key(config_keys, "rotate_right", pygame.K_e)
        self.KEY_STOP = check_exist_key(config_keys, "stop", pygame.K_SPACE)


def check_exist_str(config: dict, path: str, value: str) -> str:
    res = search_nested(config, path)
    if isinstance(res, str):
        return res
    return value

def check_exist_float(config: dict, path: str, value: float) -> float:
    res = search_nested(config, path)
    if isinstance(res, float):
        return float(res)
    return value

def check_exist_key(config: dict, path: str, value: int) -> int:
    res = search_nested(config, path)
    if isinstance(res, str):
        return pygame.key.key_code(res)
    return value

def search_nested(config: dict, path: str) -> Optional[Union[dict, str]]:
    obj: Union[dict, str] = config
    path_steps = path.split("/")
    for step in path_steps:
        if step in obj and isinstance(obj, dict):
            obj = obj[step]
        else:
            return None
    return obj
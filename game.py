#!/usr/bin/env python3

from dashboard import Dashboard, Speedometer
import numpy as np
import pygame
import cv2

import msg
from sensor import Sensor
from teleop import calculate_teleop_twist


PUBLISHER_MSGS = [msg.CMD_VEL_MSG]
SUBSCRIBER_MSGS = [msg.CMD_VEL_MSG, msg.IMAGE_MSG, msg.ODOM_MSG, msg.SCAN_MSG]

TELEOP = True

WIDTH = 900
VIEW_HEIGHT = 500
DASH_HEIGHT = 100
HEIGHT = VIEW_HEIGHT + DASH_HEIGHT

WHITE = (255, 255, 255)

FPS = 60


class Game:
    """
    Track inputs from the user and ROS and update the game window.
    """

    def __init__(self) -> None:
        self.sensors = Sensor(PUBLISHER_MSGS, SUBSCRIBER_MSGS)
        pygame.init()
        self.window = pygame.display.set_mode((WIDTH, HEIGHT))
        self.window.fill(WHITE)
        self.dash = Dashboard(WIDTH, DASH_HEIGHT, self.sensors)
        self.keys_pressed = pygame.key.get_pressed()

    def get_image(self):
        """
        Grab the newest image from the robot camera and convert it to an array.
        """
        image = self.sensors.get_msg(msg.IMAGE_MSG)
        if image is not None and isinstance(image, msg.Image):
            bgr = np.frombuffer(image.data, dtype=np.uint8).reshape(
                image.height, image.width, 3
            )
            if np.any(bgr):
                return bgr
        return None

    def send_control_input(self):
        """
        Check for user teleop input and send a velocity to cmd_vel.
        """
        if TELEOP:
            current_twist = self.sensors.get_msg(msg.CMD_VEL_MSG)
            new_twist = calculate_teleop_twist(current_twist, self.keys_pressed)
            self.sensors.publish(msg.CMD_VEL_MSG, new_twist)

    def set_background(self):
        """
        Update the game background with the newest camera image.
        """
        image = self.get_image()
        if image is not None:
            pygame_image = pygame.image.frombuffer(
                image, (image.shape[1], image.shape[0]), "RGB"
            )
            pygame_image_resized = pygame.transform.scale(
                pygame_image, (WIDTH, VIEW_HEIGHT)
            )
            self.window.blit(pygame_image_resized, (0, 0))

    def set_dash(self):
        """
        Update the dashboard with the latest sensor readings.
        """
        self.window.blit(self.dash.get_display(), (0, VIEW_HEIGHT))

    def update(self):
        """
        Update the game window.
        """
        self.keys_pressed = pygame.key.get_pressed()
        self.send_control_input()
        self.set_background()
        self.set_dash()
        pygame.display.update()


def run():
    """
    Run the main game loop.
    """
    clock = pygame.time.Clock()
    run = True
    game = Game()

    while run:
        clock.tick(FPS)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

        game.update()

    pygame.quit()


if __name__ == "__main__":
    run()
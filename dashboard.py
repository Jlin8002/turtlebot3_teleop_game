from sensor import Sensor
import pygame

FONT_SIZE = 36
BLACK = (0, 0, 0)


class DashComponent:
    def __init__(self, width, height, sensor):
        self.width = width
        self.height = height
        self.surface = pygame.Surface((width, height))
        self.sensor = sensor
        self.font = pygame.font.Font(None, FONT_SIZE)


class Dashboard(DashComponent):
    def display_velocity(self):
        width = self.width / 2
        height = self.height
        surface = pygame.Surface((width, height))

        velocity_text = self.font.render("Velocity", 1, BLACK)
        linear_text = self.font.render("Linear", 1, BLACK)
        angular_text = self.font.render("Angular", 1, BLACK)

        lin_vel_text = self.font.render()

    def draw(self):
        pass

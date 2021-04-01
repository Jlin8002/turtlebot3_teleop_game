from numpy.lib.histograms import histogram
import msg
from sensor import Sensor
import pygame

FONT_SIZE = 36
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)

SPEEDOMETER_WIDTH_RATIO = 1 / 3
SPEEDOMETER_X_RATIO = 0
GPS_WIDTH_RATIO = 1 / 3
GPS_X_RATIO = 2 / 3


class DashComponent:
    """
    A parent class for a dashboard component.
    """

    def __init__(self, width, height, sensor):
        self.width = width
        self.height = height
        self.surface = pygame.Surface((width, height))
        self.sensor = sensor
        self.font = pygame.font.Font(None, FONT_SIZE)

    def update(self):
        """
        Draw the component with updated values based on sensor readings.
        """
        pass

    def get_display(self):
        """
        Return the updated display for use by other modules.
        """
        self.update()
        return self.surface


class Speedometer(DashComponent):
    """
    A speedometer dashboard component tracking cmd_vel.
    """

    def update(self):
        """
        Draw the speedometer with the latest velocities.
        """
        self.surface.fill((255, 255, 255))

        velocity_text = self.font.render("Velocity", 1, BLACK)
        linear_text = self.font.render("Linear", 1, BLACK)
        angular_text = self.font.render("Angular", 1, BLACK)

        velocity_text_pos = velocity_text.get_rect(
            centerx=self.width * 2 / 3, centery=self.height / 4
        )
        linear_text_pos = linear_text.get_rect(
            centerx=self.width / 4, centery=self.height / 2
        )
        angular_text_pos = angular_text.get_rect(
            centerx=self.width / 4, centery=self.height * 3 / 4
        )

        current_velocity = self.sensor.get_msg(msg.CMD_VEL_MSG)

        lin_vel_text = self.font.render(f"{current_velocity.linear.x:.3f}", 1, GREEN)
        ang_vel_text = self.font.render(f"{current_velocity.angular.z:.3f}", 1, GREEN)

        lin_vel_text_pos = lin_vel_text.get_rect(
            centerx=self.width * 2 / 3, centery=self.height / 2
        )
        ang_vel_text_pos = ang_vel_text.get_rect(
            centerx=self.width * 2 / 3, centery=self.height * 3 / 4
        )

        self.surface.blit(velocity_text, velocity_text_pos)
        self.surface.blit(linear_text, linear_text_pos)
        self.surface.blit(angular_text, angular_text_pos)
        self.surface.blit(lin_vel_text, lin_vel_text_pos)
        self.surface.blit(ang_vel_text, ang_vel_text_pos)


class GPS(DashComponent):
    """
    A GPS dashboard component tracking scan and odom.
    """

    def update(self):
        """
        Draw the GPS with the latest odometry and scan information.
        """
        self.surface.fill(GREEN)


class Dashboard(DashComponent):
    """
    The main dashboard consisting of all the above components.
    """

    def __init__(self, width, height, sensor):
        super().__init__(width, height, sensor)
        self.speedometer = Speedometer(width * SPEEDOMETER_WIDTH_RATIO, height, sensor)
        self.GPS = GPS(width * GPS_WIDTH_RATIO, height, sensor)

    def update(self):
        """
        Draw the entire dashboard with the latest sensor values.
        """
        self.surface.blit(
            self.speedometer.get_display(), (self.width * SPEEDOMETER_X_RATIO, 0)
        )
        self.surface.blit(self.GPS.get_display(), (self.width * GPS_X_RATIO, 0))
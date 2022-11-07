from msg import ROSMsg, Odometry, Twist
from sensor import Sensor
import pygame

from typing import Optional

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

    def __init__(self, width: int, height: int, sensor: Sensor, msg: Optional[ROSMsg] = None):
        self.width = width
        self.height = height
        self.surface = pygame.Surface((width, height))
        self.sensor = sensor
        self.msg = msg
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
        self.surface.fill(WHITE)

        velocity_text = self.font.render("Velocity", True, BLACK)
        linear_text = self.font.render("Linear", True, BLACK)
        angular_text = self.font.render("Angular", True, BLACK)

        velocity_text_pos = velocity_text.get_rect(
            centerx=self.width * 2 / 3, centery=self.height / 4
        )
        linear_text_pos = linear_text.get_rect(
            centerx=self.width / 4, centery=self.height / 2
        )
        angular_text_pos = angular_text.get_rect(
            centerx=self.width / 4, centery=self.height * 3 / 4
        )

        assert self.msg is not None

        current_velocity = self.sensor.get_msg(self.msg)

        assert isinstance(current_velocity, Twist)

        lin_vel_text = self.font.render(f"{current_velocity.linear.x:.3f}", True, GREEN)
        ang_vel_text = self.font.render(f"{current_velocity.angular.z:.3f}", True, GREEN)

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
    A GPS dashboard component tracking odom.
    """

    def update(self):
        """
        Draw the GPS with the latest odometry information.
        """
        self.surface.fill(GREEN)
        x_text = self.font.render("x", True, BLACK)
        y_text = self.font.render("y", True, BLACK)
        z_text = self.font.render("z", True, BLACK)

        x_text_pos = x_text.get_rect(
            centerx=self.width / 4, centery=self.height / 4
        )
        y_text_pos = y_text.get_rect(
            centerx=self.width / 4, centery=self.height / 2
        )
        z_text_pos = z_text.get_rect(
            centerx=self.width / 4, centery=self.height * 3 / 4
        )

        assert self.msg is not None

        current_odom = self.sensor.get_msg(self.msg)

        assert isinstance(current_odom, Odometry)

        x_pos_text = self.font.render(f"{current_odom.pose.pose.position.x:.3f}", True, WHITE)
        y_pos_text = self.font.render(f"{current_odom.pose.pose.position.y:.3f}", True, WHITE)
        z_pos_text = self.font.render(f"{current_odom.pose.pose.position.z:.3f}", True, WHITE)

        x_pos_text_pos = x_pos_text.get_rect(
            centerx=self.width * 3 / 5, centery=self.height / 4
        )
        y_pos_text_pos = y_pos_text.get_rect(
            centerx=self.width * 3 / 5, centery=self.height / 2
        )
        z_pos_text_pos = z_pos_text.get_rect(
            centerx=self.width * 3 / 5, centery=self.height * 3 / 4
        )

        self.surface.blit(x_text, x_text_pos)
        self.surface.blit(y_text, y_text_pos)
        self.surface.blit(z_text, z_text_pos)
        self.surface.blit(x_pos_text, x_pos_text_pos)
        self.surface.blit(y_pos_text, y_pos_text_pos)
        self.surface.blit(z_pos_text, z_pos_text_pos)

class Map(DashComponent):
    """
    A map dashboard component tracking scan.
    """
    def update(self):
        """
        Draw the map with the latest scan information.
        """
        pass


class Dashboard(DashComponent):
    """
    The main dashboard consisting of all the above components.
    """

    def __init__(self, width, height, sensor, msg_speedometer, msg_gps):
        super().__init__(width, height, sensor)
        self.speedometer = Speedometer(int(width * SPEEDOMETER_WIDTH_RATIO), height, sensor, msg_speedometer)
        self.GPS = GPS(int(width * GPS_WIDTH_RATIO), height, sensor, msg_gps)

    def update(self):
        """
        Draw the entire dashboard with the latest sensor values.
        """
        self.surface.blit(
            self.speedometer.get_display(), (self.width * SPEEDOMETER_X_RATIO, 0)
        )
        self.surface.blit(self.GPS.get_display(), (self.width * GPS_X_RATIO, 0))
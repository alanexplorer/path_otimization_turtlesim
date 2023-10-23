import numpy as np
from math import atan2, pi
from turtlesim.msg import Pose

class Controller:

    def __init__(self, k_p: float, k_alpha: float, k_beta: float):
        self.update_parameters(k_p, k_alpha, k_beta)

        self.__reset()

    def update_parameters(self, k_p: float, k_alpha: float, k_beta: float):
        self.kp = k_p
        self.kalpha = k_alpha
        self.kbeta = k_beta

    def __reset(self):
        self.error_dist = 0
        self.error_angle = 0
        self.error_angle_alpha = 0
        self.error_angle_beta = 0
        self.__linear_velocity = 0
        self.__angular_velocity = 0
        self.threshold_linear = 0.3
        self.threshold_angular = 0.2
        self.__tinyAdjust = False

    def approach(self, robot: Pose, target: Pose):

        self.error_dist = ((target.x - robot.x) ** 2 +
                           (target.y - robot.y) ** 2) ** 0.5
        self.error_angle = self.normalize_angle((atan2(target.y - robot.y, target.x - robot.x) - robot.theta))

        robot.theta = self.normalize_angle(robot.theta)
        target.theta = self.normalize_angle(target.theta)

        # Todo analisar esta logica
        # Abaixo do threshold_linear nao ha necessidade de KRho e Kalpha
        # Robo Instavel na transicao deste if
        if self.error_dist < self.threshold_linear + 0.05:
            self.__tinyAdjust = True

        if self.__tinyAdjust:
            self.error_angle_alpha = 0
            self.error_dist = 0
            # turn gain
            self.error_angle_beta = 5 * self.normalize_angle(target.theta - robot.theta)
            if (self.normalize_angle(target.theta - robot.theta) < pi):
                self.error_angle_beta *= -1

        else:
            self.error_angle_alpha = self.normalize_angle(
                atan2(target.y - robot.y, target.x - robot.x) - robot.theta)
            self.error_angle_beta = self.normalize_angle(
                target.theta - atan2(target.y - robot.y, target.x - robot.x))

        if ((abs(self.normalize_angle(target.theta-robot.theta)) <= self.threshold_angular) and (self.error_dist <= self.threshold_linear)):
            self.__reset()
            return True

        else:
            # Adjustment of the reference signal for backward movement if the endpoint is behind the robot
            if self.is_point_behind_robot():
                self.error_dist *= -1
                self.error_angle = self.normalize_angle(self.error_angle + pi)
                self.error_angle_alpha = self.normalize_angle(
                    self.error_angle_alpha + pi)
                self.error_angle_beta = self.normalize_angle(
                    self.error_angle_beta + pi)

            self.__calculate_velocity()

        return False

    def get_velocity_command(self):
        return self.__linear_velocity, self.__angular_velocity

    def __calculate_velocity(self):
        self.__linear_velocity = self.kp * self.error_dist
        self.__angular_velocity = self.kalpha * \
            self.error_angle_alpha - self.kbeta * self.error_angle_beta

        if (self.kalpha * self.error_angle_alpha == 0):
            self.__angular_velocity = max(
                min(self.__angular_velocity, 0.25), -0.25)

    def is_point_behind_robot(self):

        if ((((self.error_angle_alpha >= -pi and self.error_angle_alpha <= -pi/2) or ((self.error_angle_alpha) >= pi/2 and (self.error_angle_beta) <= pi))) and not self.__tinyAdjust):
            return True
        return False

    def set_threshold(self, linear, angular):
        self.threshold_linear = linear
        self.threshold_angular = angular

    def normalize_angle(self, angle):
            angle_degree = angle * 180 / pi

            angle_degree = (angle_degree % 360)
            if angle_degree > 180:
                angle_degree = angle_degree -360
            
            return angle_degree * pi /180
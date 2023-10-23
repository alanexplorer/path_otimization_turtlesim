import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import rclpy
from path_optimization.turtle_simulation.turtle_sim import TurtleSimManager
from path_optimization.screen_manager.screen_manager import ScreenManager
import random
from deap import base, creator, tools, algorithms
import string
import time


def main(args=None):
    rclpy.init(args=args)

        # self.controller = Controller(1.0, 1.05, 0.5)

    k = 1.0
    k_alpha = 1.05
    k_beta = 0.5

    # k = 2.364942545424096
    # k_alpha =0.1542182518462227
    # k_beta = 2.15192323003995

    robot = TurtleSimManager("robot_test", k, k_alpha, k_beta)
    robot.spawn(1.0, 5.0, 0.0)
    robot.add_subscription()
    robot.pen_request()

    while rclpy.ok():

        distance_computed = robot.compute_dist

        rclpy.spin_once(robot)

        if robot.arrived and distance_computed:
            break

    print(robot.distance)

    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
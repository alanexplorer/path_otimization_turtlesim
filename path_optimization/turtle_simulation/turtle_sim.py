from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Kill
from turtlesim.srv import Spawn
from turtlesim.srv import SetPen
from path_optimization.controller.controller import Controller
import numpy as np
from scipy.spatial.distance import euclidean
from fastdtw import fastdtw
from math import pi
import random

class TurtleSimManager(Node):

    def __init__(self, name, k, k_alpha, k_beta):

        self.name = name

        super().__init__(f'turtlesim_manager_{name}')

        self.cmd_msg = Twist()

        self.controller = Controller(k, k_alpha, k_beta)
        self.arrived = False
        self.compute_dist = False
        self.collided = False

        self.distance = None
        self.initial_pose = Pose()
        self.current_path = []

        self.target_list = []
        self.target_size = len(self.target_list)

        self.target = Pose()
        self.target.x = 9.0
        self.target.y = 1.0
        self.target.theta = 90.0*pi/180
        self.target_list.append(self.target)

        self.target = Pose()
        self.target.x = 9.0
        self.target.y = 10.0
        self.target.theta = 180.0*pi/180
        self.target_list.append(self.target)

        self.target = Pose()
        self.target.x = 1.0
        self.target.y = 9.0
        self.target.theta = -90.0*pi/180
        self.target_list.append(self.target)

        self.target = Pose()
        self.target.x = 1.0
        self.target.y = 1.0
        self.target.theta = 0.0*pi/180
        self.target_list.append(self.target)


    def add_subscription(self):

        self.publisher = self.create_publisher(Twist, f'/{self.name}/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, f'/{self.name}/pose', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):

        if(not self.is_collision(msg)):

            if(not self.target_list):
                self.arrived = True

            if(self.target_list and not self.arrived):
                
                self.current_path.append([msg.x, msg.y])

                passed = self.controller.approach(msg, self.target_list[0])

                command = self.controller.get_velocity_command()
                self.cmd_msg.linear.x = float(command[0])
                self.cmd_msg.angular.z = float(command[1])

                self.publisher.publish(self.cmd_msg)

                if passed:
                    self.target_list.pop(0)

            else:
                self.get_logger().info(f' {self.name} arrived. ')
        else:
            self.collided = True

    def is_collision(self, pose):
            return pose.x <= 0 or pose.x >= 11.0 or pose.y <= 0 or pose.y >= 11.0

    def pen_request(self):

        srv_pen = self.create_client(SetPen, f'/{self.name}/set_pen')

        while not srv_pen.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('service not available, waiting again...')
        
        req = SetPen.Request()

        req.r = random.randint(0, 255)
        req.g = random.randint(0, 255)
        req.b = random.randint(0, 255)
        req.width = 2
        srv_pen.call_async(req)

    def kill_request(self):

        srv_kill = self.create_client(Kill, '/kill')
        while not srv_kill.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('service not available, waiting again...')
        
        req = Kill.Request()
        req.name = self.name
        srv_kill.call_async(req)

    def spawn(self, x: float, y: float, theta: float):

        self.initial_pose.x = x
        self.initial_pose.y = y
        self.initial_pose.theta = theta

        srv_spawn = self.create_client(Spawn, '/spawn')
        while not srv_spawn.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        req = Spawn.Request()

        req.name = self.name
        req.x = x
        req.y = y
        req.theta = theta
        srv_spawn.call_async(req)
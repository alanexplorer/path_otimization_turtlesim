from rclpy.node import Node
from turtlesim.srv import Kill
from std_srvs.srv import Empty

class ScreenManager(Node):

    def __init__(self):

        super().__init__('screen_manager')

    def kill(self):

        srv_kill = self.create_client(Kill, '/kill')
        while not srv_kill.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('service not available, waiting again...')
        
        req = Kill.Request()
        req.name = 'turtle1'
        srv_kill.call_async(req)

    def clear(self):

        srv_clear = self.create_client(Empty, '/clear')
        while not srv_clear.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        srv_clear.call_async(Empty.Request())

    def reset(self):

        srv_reset = self.create_client(Empty, '/reset')
        while not srv_reset.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        srv_reset.call_async(Empty.Request())

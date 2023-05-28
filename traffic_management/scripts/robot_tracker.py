import rclpy
from rclpy.node import Node

from math import sqrt, pow

from std_msgs.msg import String
from nav_msgs.msg import Odometry

class RobotTracker(Node):

    def __init__(self):
        super().__init__('robot_tracker')
        
        self.MIR_01_subscription = self.create_subscription(
            Odometry,
            '/MIR_01/odom',
            self.MIR_01_listener_callback,
            10)
        self.MIR_01_subscription  # prevent unused variable warning
        self.MIR_01_distance = 0.0
        self.MIR_01_x = None
        self.MIR_01_y = None
        self.MIR_01_counter = 0

        self.MIR_02_subscription = self.create_subscription(
            Odometry,
            '/MIR_02/odom',
            self.MIR_02_listener_callback,
            10)
        self.MIR_02_subscription  # prevent unused variable warning
        self.MIR_02_distance = 0.0
        self.MIR_02_x = None
        self.MIR_02_y = None
        self.MIR_02_counter = 0

    def MIR_01_listener_callback(self, msg):
        x = round(msg.pose.pose.position.x, 2)
        y = round(msg.pose.pose.position.y, 2)
        self.MIR_01_counter += 1

        if (self.MIR_01_x == None or self.MIR_01_y == None):
            self.MIR_01_x = x
            self.MIR_01_y = y
        
        else: 
            dist = sqrt(pow((self.MIR_01_x - x), 2) + pow((self.MIR_01_y - y), 2))
            self.MIR_01_x = x
            self.MIR_01_y = y
            self.MIR_01_distance += dist
            if (self.MIR_01_counter % 100 == 0):
                self.get_logger().info('MiR 01 moved ' + str(self.MIR_01_distance))

    def MIR_02_listener_callback(self, msg):
        x = round(msg.pose.pose.position.x, 2)
        y = round(msg.pose.pose.position.y, 2)
        self.MIR_02_counter += 1

        if (self.MIR_02_x == None or self.MIR_02_y == None):
            self.MIR_02_x = x
            self.MIR_02_y = y
        
        else: 
            dist = sqrt(pow((self.MIR_02_x - x), 2) + pow((self.MIR_02_y - y), 2))
            self.MIR_02_x = x
            self.MIR_02_y = y
            self.MIR_02_distance += dist
            if (self.MIR_02_counter % 100 == 0):
                self.get_logger().info('MiR 02 moved ' + str(self.MIR_02_distance))
        
def main(args=None):
    rclpy.init(args=args)

    print('Subscribing...')
    robot_tracker = RobotTracker()
    print('Subscribed!')

    rclpy.spin(robot_tracker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
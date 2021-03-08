import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist
from std_msgs.msg import String


class Controller(Node):
    LIN_VELOCITY = 0.2
    ANG_VELOCITY = 0.2

    def __init__(self):
        super().__init__("controller")
        self.publisher_ = self.create_publisher(Twist, 'velocity_topic', 2)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.LIN_VELOCITY
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.ANG_VELOCITY

        self.publisher_.publish(msg)
        self.get_logger().info("msg")


def main(args=None):
    rclpy.init(args=args)

    publisher = Controller()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
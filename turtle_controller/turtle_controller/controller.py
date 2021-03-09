import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType
from turtle_controller.key_capturer import KeyCapturer
import threading

class Publisher(Node):
    ANG_VEL = 0.5
    LIN_VEL = 0.5

    def __init__(self):
        super().__init__("controller")
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 1)
        self.declare_parameter('forward', '<Up>')
        self.declare_parameter('left', '<Left>')
        self.declare_parameter('right', '<Right>')
        self.declare_parameter('backwards', '<Down>')

    def publish_(self, lin_vel, ang_vel):
        msg = Twist()
        msg.linear.x = lin_vel
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = ang_vel

        self.publisher_.publish(msg)

    def rotate_right(self, event):
        self.publish_(0.0, -self.ANG_VEL)

    def rotate_left(self, event):
        self.publish_(0.0, self.ANG_VEL)

    def go_forward(self, event):
        self.publish_(self.LIN_VEL, 0.0)

    def go_backwards(self, event):
        self.publish_(-self.LIN_VEL, 0.0)

def main(args=None):
    rclpy.init(args=args)

    publisher = Publisher()
    key_capturer = KeyCapturer(publisher)
    th1 = threading.Thread(target=rclpy.spin, args=(key_capturer.publisher,))
    th1.start()
    key_capturer.run()



    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

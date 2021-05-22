import rclpy
from rclpy.node import Node
import numpy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Pose ,PoseStamped
from math import cos, sin ,asin , acos, atan2
from lab5.functions import *
from rclpy.clock import ROSClock
from visualization_msgs.msg import MarkerArray, Marker
from lab4_service.srv import InterpolationPoint
from lab5.interpolation_methods_point import *

class Draw_elipse(Node):

    def __init__(self):
        super().__init__('draw_elipse')
        self.subscriber = self.create_subscription(PoseStamped, "interpolator_point", self.listener_callback, 1)
        self.publisher = self.create_publisher(PoseStamped, 'interpolator_point', 1)

    def listener_callback(self, msg):
        x = msg.position[0]
        y = msg.position[1]
        z = msg.position[2]
        ##a - promień większy
        ## b - promień mniejszy
        a = 1
        b = 1
        self.calc_elipse(a,b,x,y)

    def calc_elipse(self,a,b,nx,ny):
        ##a - promień większy
        ## b - promień mniejszy
        angle = atan2(ny, nx)
        t = angle + pi/100
        x = a*cos(t)
        y = b * sin(t)

    def inv_calc_elipse(self,a,nx):
        return(acos(nx/a))


def main(args=None):
    rclpy.init(args=args)

    draw = Draw_elipse()

    rclpy.spin(draw)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    draw.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

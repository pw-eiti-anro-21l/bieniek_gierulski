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

class Draw_figure(Node):

    def __init__(self):
        super().__init__('draw_figure')
        self.subscriber = self.create_subscription(PoseStamped, "interpolator_point", self.listener_callback, 1)
        self.current_xyz = [0,0,0]
    def listener_callback(self, msg):
        x = msg.position[0]
        y = msg.position[1]
        z = msg.position[2]
        self.current_xyz = [x,y,z]
        ##a - promień większy
        ## b - promień mniejszy

    def calc_elipse(self,a,b):
        ##a - promień większy
        ## b - promień mniejszy
        nx = self.current_xyz[0]
        ny = self.current_xyz[1]
        angle = atan2(ny, nx)
        t = angle + pi/100
        x = a*cos(t)
        y = b * sin(t)
        return [x,y]

    def calc_corners(self,a,b):
        ## a - krutszy bok
        ## b - dluzszy bok
        nx = self.current_xyz[0]
        ny = self.current_xyz[1]
        x2 = nx + b
        y2 = ny
        x3 = x2
        y3 = y2 + a
        x4 = nx
        y4 = y3
        a1 = [nx,ny]
        a2 = [x2, y2]
        a3 = [x3, y3]
        a4 = [x4, y4]
        return [a1,a2,a3,a4]

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

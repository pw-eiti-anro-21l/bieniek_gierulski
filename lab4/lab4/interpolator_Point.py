import rclpy
from rclpy.node import Node
import numpy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Pose, PoseStamped
from math import cos, sin
from lab3_essential.functions import *
from rclpy.clock import ROSClock
from visualization_msgs.msg import MarkerArray, Marker
from lab4_service.srv import Interpolation
from lab4.interpolation_methods_point import *

class Oint_Pub(Node):

    def __init__(self):
        super().__init__('oint_pub')
        self.srv = self.create_service(Interpolation_Point, 'Interpolation_Point', self.interpolation_callback)
        self.publisher = self.create_publisher(PoseStamped, 'oint_pub', 1)
        self.joint_positions = [0.0, 0.0, 0.0]
        self.joint_rotation = [0.0, 0.0, 0.0]

    def interpolation_callback(self, request, response):
        j1 = [request.x_pos, request.y_pos, request.z_pos]
        j2 = [request.roll, request.pitch, request.yaw]
        time = request.time
        if request.method == "linear":
            interpolator = LinearInterpolatorPoint()
            interpolator.interpolate(self.joint_positions, j1,j2, time, self.publisher)
        elif request.method == "cubic":
            interpolator = CubicInterpolatorPoint()
            interpolator.interpolate(self.joint_positions, j1,j2, time, self.publisher)

        response.result = "Done"
        return response


def main(args=None):
    rclpy.init(args=args)

    no_kdl = Oint_Pub()

    rclpy.spin(no_kdl)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    no_kdl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

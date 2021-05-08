import rclpy
from rclpy.node import Node
import numpy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Pose ,PoseStamped
from math import cos, sin
from lab3_essential.functions import *
from rclpy.clock import ROSClock
from visualization_msgs.msg import MarkerArray, Marker
from lab4_point_service.srv import InterpolationPoint
from lab4.interpolation_methods_point import *

class Oint_Pub(Node):

    def __init__(self):
        super().__init__('oint_pub')
        self.srv = self.create_service(InterpolationPoint, 'InterpolationPoint', self.interpolation_callback)
        self.subscription = self.create_subscription(
            PoseStamped,
            'pose',
            self.listener_callback, 1)

        self.publisher = self.create_publisher(PoseStamped, 'oint_pub', 1)
        self.joint_positions = [0.0, 0.0, 0.0]
        self.joint_rotation = [0.0, 0.0, 0.0]

        msg = PoseStamped()
        msg.header.stamp = ROSClock().now().to_msg()
        self.publisher.publish(msg)

    def interpolation_callback(self, request, response):
        j1 = [request.x_pos, request.y_pos, request.z_pos]
        j2 = [request.roll, request.pitch, request.yaw]
        time = request.time
        if request.method == "linear":
            interpolator = LinearInterpolatorPoint()
            interpolator.interpolate(self.joint_positions, j1,self.joint_rotation , j2, time, self.publisher)
        elif request.method == "cubic":
            interpolator = CubicInterpolatorPoint()
            interpolator.interpolate(self.joint_positions, j1,self.joint_rotation , j2, time, self.publisher)

        response.result = "Done"
        return response

    def listener_callback(self, msg):
        pose = msg.position
        orientation = msg.pose.orientation
        self.joint_positions[0] = pose[0]
        self.joint_positions[1] = pose[1]
        self.joint_positions[2] = pose[2]
        self.joint_rotation[0] = orientation[0]
        self.joint_rotation[1] = orientation[1]
        self.joint_rotation[2] = orientation[2]

def main(args=None):
    rclpy.init(args=args)

    oint_pub = Oint_Pub()

    rclpy.spin(oint_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    oint_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

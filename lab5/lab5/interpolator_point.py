import rclpy
from rclpy.node import Node
import numpy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Pose ,PoseStamped
from math import cos, sin
from lab5.functions import *
from rclpy.clock import ROSClock
from visualization_msgs.msg import MarkerArray, Marker
from lab4_service.srv import InterpolationPoint
from lab5.interpolation_methods_point import *

class Oint_Pub(Node):

    def __init__(self):
        super().__init__('interpolator_point')
        self.srv = self.create_service(InterpolationPoint, 'InterpolationPoint', self.interpolation_callback)
        self.publisher = self.create_publisher(PoseStamped, 'interpolator_point', 1)
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback, 1)
        self.positions = [2.0, 0.0, 0.4]
        self.rotations = [0.0, 0.0, 0.0]


        # Publish once
        msg = PoseStamped()
        msg.header.stamp = ROSClock().now().to_msg()
        msg.header.frame_id = "base"
        msg.pose.position.x = 0.0
        msg.pose.position.z = 0.0
        msg.pose.position.y = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        self.publisher.publish(msg)

    def listener_callback(self, msg):
        self.positions[0] = msg.position[0]
        self.positions[1] = msg.position[1]
        self.positions[2] = msg.position[2]

    def interpolation_callback(self, request, response):
        j1 = [request.x_pos, request.y_pos, request.z_pos]
        j2 = [request.roll, request.pitch, request.yaw]
        time = request.time
        if request.method == "linear":
            interpolator = LinearInterpolatorPoint(self.positions)
            interpolator.interpolate(self.positions, j1,self.rotations , j2, time, self.publisher)
        elif request.method == "cubic":
            interpolator = CubicInterpolatorPoint(self.positions)
            interpolator.interpolate(self.positions, j1,self.rotations , j2, time, self.publisher)



        self.positions = j1
        self.rotations = j2

        response.result = "Done"
        return response

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

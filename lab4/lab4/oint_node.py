import rclpy
from rclpy.node import Node
import numpy
from sensor_msgs.msg import JointState
from lab4.functions import *
from geometry_msgs.msg import Point, Pose, PoseStamped
from rclpy.clock import ROSClock
from visualization_msgs.msg import MarkerArray, Marker
from functions import *
from interpolation_methods import *

class oint_PUB(Node):

    def __init__(self):
        super().__init__('oint_pub')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback, 1)
        self.publisher = self.create_publisher(PoseStamped, 'oint_pub', 1)
        self.publisher2 = self.create_publisher(MarkerArray, "trajectory", 1)
        self.DH = get_dh_table()
        self.markers = MarkerArray()
        self.start_orient = [0.0, 0.0, 0.0]
        self.start_pose = [0.0, 0.0, 0.0]

       # publish 1 time
        msg = JointState()
        msg.header.stamp = ROSClock().now().to_msg()
        msg.name = ["elevator_to_rotator", "rotator_to_rotator2", "rotator2_to_arm"]
        msg.position = self.joint_positions
        self.publisher.publish(msg)

    def listener_callback(self, msg):
        self.joint_positions[0] = msg.position[0]
        self.joint_positions[1] = msg.position[1]
        self.joint_positions[2] = msg.position[2]

    def interpolation_callback(self, request, response):
        j1 = [request.joint1_pos, request.joint2_pos, request.joint3_pos]
        time = request.time
        if request.method == "linear":
            interpolator = LinearInterpolator()
            interpolator.interpolate(self.joint_positions, j1, time, self.publisher)
        elif request.method == "cubic":
            interpolator = CubicInterpolator()
            interpolator.interpolate(self.joint_positions, j1, time, self.publisher)

        response.result = "Done"
        return response


    def linear_interpolation(self):
        pose_x = self.start_pose[0]
        pose_y = self.start_pose[1]
        pose_z = self.start_pose[2]

        ort_roll = self.start_orient[0]
        ort_pitch = self.start_orient[1]
        ort_yaw = self.start_orient[2]

        marker = Marker()
        marker.header.stamp = ROSClock().now().to_msg()
        marker.header.frame_id = "/base"
        marker.type = 2
        marker.pose = Pose
        marker.id = len(self.markers.markers) + 1
        marker.action = Marker.ADD
        marker.type = Marker.SPHERE
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

def main(args=None):
    rclpy.init(args=args)

    oint_pub = oint_PUB()

    rclpy.spin(oint_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    oint_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

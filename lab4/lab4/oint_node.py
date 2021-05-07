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

class Oint_Pub(Node):

    def __init__(self):
        super().__init__('oint_pub')
        self.srv = self.create_service(Interpolation, 'Interpolation', self.interpolation_callback)
        self.publisher = self.create_publisher(PoseStamped, 'oint_pub', 1)
        self.publisher2 = self.create_publisher(MarkerArray, "trajectory", 1)
        self.markers = MarkerArray()
        self.joint_positions = [0.0, 0.0, 0.0]
        self.joint_rotation = [0.0, 0.0, 0.0]

    def interpolation_callback(self, request, response):
        j1 = [request.joint1_pos, request.joint2_pos, request.joint3_pos]
        time = request.time
        if request.method == "linear":
            interpolator = LinearInterpolatorPoint()
            interpolator.interpolate(self.joint_positions, j1, time, self.publisher)
        elif request.method == "cubic":
            interpolator = CubicInterpolatorPoint()
            interpolator.interpolate(self.joint_positions, j1, time, self.publisher)

        response.result = "Done"
        return response

    def listener_callback(self, msg):
        position= self.calculate(msg.position)
        point = Point()
        point.x = position[0]
        point.y = position[1]
        point.z = position[2]
        pose = Pose()
        pose.position = point
        pose_st = PoseStamped()
        pose_st.pose = pose
        pose_st.header.stamp = ROSClock().now().to_msg()
        pose_st.header.frame_id = "base"

        marker = Marker()
        marker.header.stamp = ROSClock().now().to_msg()
        marker.header.frame_id = "/base"
        marker.type = 2
        marker.pose = pose
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

        self.markers.markers.append(marker)
        self.publisher2.publish(self.markers)
        self.publisher.publish(pose_st)

class LinearInterpolatorPoint:

    def __init__(self):
        self.DELAY_TIME = 0.05
        self.start_time = 0
        self.states_done = []  # if len == 3 -> all 3 interpolations done
        self.joints_actual_positions = [None, None, None]

    def publish_messages(self):
        # Joint states
        msg = JointState()
        msg.header.stamp = ROSClock().now().to_msg()
        msg.name = ["elevator_to_rotator", "rotator_to_rotator2", "rotator2_to_arm"]
        msg.position = self.joints_actual_positions
        self.publisher.publish(msg)

    def interpolate(self, j0, j1, time, publisher):
        # j0 - initial joint positions
        # j1 - final joint positions
        self.publisher = publisher

        self.calc(j0[0], j1[0], self.start_time, self.start_time + time, self.DELAY_TIME, 0)
        self.calc(j0[1], j1[1], self.start_time, self.start_time + time, self.DELAY_TIME, 1)
        self.calc(j0[1], j1[1], self.start_time, self.start_time + time, self.DELAY_TIME, 2)

        while len(self.states_done) != 3:
            self.publish_messages()

    def calc(self, x0, x1, t0, t1, t_delta, joint_nr):
        new_value = x0 + ((x1 - x0) / (t1 - t0)) * (t_delta - t0)
        self.joints_actual_positions[joint_nr] = new_value

        if t_delta >= t1:
            self.states_done.append(True)
        else:
            t = Timer(self.DELAY_TIME, self.calc, [x0, x1, t0, t1, t_delta + self.DELAY_TIME, joint_nr])
            t.start()

def main(args=None):
    rclpy.init(args=args)

    no_kdl = No_KDL()

    rclpy.spin(no_kdl)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    no_kdl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

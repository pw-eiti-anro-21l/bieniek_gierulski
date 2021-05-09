import rclpy
from rclpy.node import Node
import numpy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Pose, PoseStamped
from math import cos, sin
from lab3_essential.functions import *
from rclpy.clock import ROSClock
from visualization_msgs.msg import MarkerArray, Marker


class No_KDL(Node):

    def __init__(self):
        super().__init__('no_kdl')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback, 1)
        self.publisher = self.create_publisher(PoseStamped, 'non_kdl', 1)
        self.publisher2 = self.create_publisher(MarkerArray, "trajectory", 1)
        self.markers = MarkerArray()
        self.DH = get_dh_table()


    def calculate(self, msg):
        matrices_list = []
        self.DH[1][1] = msg[0]
        self.DH[2][3] = msg[1]
        self.DH[3][3] = msg[2]

        for row in self.DH:
            result = []
            result.append([cos(row[3]), -1 * sin(row[3]), 0, row[0]])
            result.append([sin(row[3]) * cos(row[2]), cos(row[3]) * cos(row[2]), -1 * sin(row[2]), -1 * sin(row[2]) * row[1]])
            result.append([sin(row[3]) * sin(row[2]), cos(row[3]) * sin(row[2]), cos(row[2]), cos(row[2]) * row[1]])
            result.append([0, 0, 0, 1])
            matrices_list.append(result)
        total_result = numpy.dot(matrices_list[0], matrices_list[1])
        for matrix in matrices_list[2:-1]:
            total_result = numpy.dot(total_result, matrix)
        position = numpy.dot(total_result, [self.DH[-1][0], 0, 0, 1])
        return position[0:3], total_result



    def listener_callback(self, msg):
        position, matrix = self.calculate(msg.position)
        point = Point()
        point.x = position[0]
        point.y = position[1]
        point.z = position[2]
        quaternion = rot_matrix_to_quaternion(matrix)
        pose = Pose()
        pose.position = point
        pose.orientation = quaternion
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
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.markers.markers.append(marker)
        self.publisher2.publish(self.markers)
        self.publisher.publish(pose_st)



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

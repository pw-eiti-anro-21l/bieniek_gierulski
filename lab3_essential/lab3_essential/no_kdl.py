import rclpy
from rclpy.node import Node
import numpy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Pose, PoseStamped
from math import cos, sin
from lab3_essential.functions import *
from rclpy.clock import ROSClock


class No_KDL(Node):

    def __init__(self):
        super().__init__('no_kdl')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback, 1)
        self.publisher = self.create_publisher(PoseStamped, 'non_kdl', 1)
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
        if self.check_joint_positions(msg):
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
            self.publisher.publish(pose_st)

    def check_joint_positions(self, msg):
        if msg.position[0] > 1.01 or msg.position[0] < 0:
            self.get_logger().error("Joint 1 invalid position")
            return False
        elif msg.position[1] > 6.29 or msg.position[1] < 0:
            self.get_logger().error("Joint 2 invalid position")
            return False
        elif msg.position[2] > 1.9 or msg.position[2] < -1.9:
            self.get_logger().error("Joint 3 invalid position")
            return False
        return True


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

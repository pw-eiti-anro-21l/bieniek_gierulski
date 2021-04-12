import rclpy
from rclpy.node import Node
import numpy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, Point, Pose
from ament_index_python.packages import get_package_share_directory
import os
from math import cos, sin


class No_KDL(Node):

    def __init__(self):
        super().__init__('no_kdl')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback, 1)
        self.publisher_ = self.create_publisher(JointState, 'topic', 1)
        self.DH = self.get_dh_table()

    def get_dh_table(self):
        name= os.path.join(
            get_package_share_directory('lab3_essential'),
            "dh.txt")
        DH1 = []
        with open(name) as file:
            for line in file:
                line = line.split()
                DH1.append(line)
        del DH1[0]
        DH = []
        for row in DH1:
            new_row = []
            for param in row:
                try:
                    new_row.append(float(param))
                except:
                    new_row.append(param)
            DH.append(new_row)
        return DH

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
        return position[0:3]



    def listener_callback(self, msg):
        position = self.calculate(msg.position)
        print(position)


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

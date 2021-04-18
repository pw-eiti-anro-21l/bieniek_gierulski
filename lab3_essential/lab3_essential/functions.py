from math import *
from ament_index_python.packages import get_package_share_directory
import os
from geometry_msgs.msg import Quaternion


def get_dh_table():
    name = os.path.join(
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


def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
    qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2)
    qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2)
    qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def rot_matrix_to_rpy(matrix):
    yaw = atan2(matrix[1][0], matrix[0][0])
    pitch = atan2(-1 * matrix[2][0], sqrt(matrix[2][1] ** 2 + matrix[2][2] ** 2))
    roll = atan2(matrix[2][1], matrix[2][2])
    return roll, pitch, yaw


def rot_matrix_to_quaternion(matrix):
    roll, pitch, yaw = rot_matrix_to_rpy(matrix)
    quaternion = euler_to_quaternion(roll, pitch, yaw)
    return quaternion

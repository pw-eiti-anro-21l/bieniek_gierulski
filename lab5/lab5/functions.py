from math import *
from ament_index_python.packages import get_package_share_directory
import os
from geometry_msgs.msg import Quaternion
import yaml
import numpy


def get_dh_table():
    name = os.path.join(
        get_package_share_directory('lab5'),
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


def load_yaml():
    with open(os.path.join(get_package_share_directory('lab5'), 'data.yaml'), 'r') as file:
        yamll = yaml.load(file)
        return yamll

def forward_kin_calc(DH, msg):
    # msg - joint states
    matrices_list = []
    DH[1][1] = msg[0]
    DH[2][3] = msg[1]
    DH[3][3] = msg[2]

    for row in DH:
        result = []
        result.append([cos(row[3]), -1 * sin(row[3]), 0, row[0]])
        result.append(
            [sin(row[3]) * cos(row[2]), cos(row[3]) * cos(row[2]), -1 * sin(row[2]), -1 * sin(row[2]) * row[1]])
        result.append([sin(row[3]) * sin(row[2]), cos(row[3]) * sin(row[2]), cos(row[2]), cos(row[2]) * row[1]])
        result.append([0, 0, 0, 1])
        matrices_list.append(result)
    total_result = numpy.dot(matrices_list[0], matrices_list[1])
    for matrix in matrices_list[2:-1]:
        total_result = numpy.dot(total_result, matrix)
    position = numpy.dot(total_result, [DH[-1][0], 0, 0, 1])
    return position[0:3], total_result

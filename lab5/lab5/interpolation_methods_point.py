from rclpy.clock import ROSClock
from geometry_msgs.msg import Point, Pose, PoseStamped
from threading import Timer
from lab5.functions import *


class LinearInterpolatorPoint:

    def __init__(self, actual):
        self.DELAY_TIME = 0.05
        self.start_time = 0
        self.states_done = []  # if len == 6 -> all 3 interpolations done
        self.Point_pose = actual
        self.Point_rotate = [0.0, 0.0, 0.0]

    def publish_messages(self):
        msg = PoseStamped()
        msg.header.stamp = ROSClock().now().to_msg()
        msg.header.frame_id = "base"
        msg.pose.position.x = self.Point_pose[0]
        msg.pose.position.y = self.Point_pose[1]
        msg.pose.position.z = self.Point_pose[2]
        quaternion = euler_to_quaternion(self.Point_rotate[0], self.Point_rotate[1], self.Point_rotate[2])
        msg.pose.orientation = quaternion
        self.publisher.publish(msg)

    def interpolate(self, j0, j1, j2, j3, time, publisher):
        # j0 - initial cords
        # j1 - final cords
        # j2 - initial rpy
        # j3 - final rpy
        self.publisher = publisher

        self.calc(j0[0], j1[0], self.start_time, self.start_time + time, self.DELAY_TIME, 0)
        self.calc(j0[1], j1[1], self.start_time, self.start_time + time, self.DELAY_TIME, 1)
        self.calc(j0[2], j1[2], self.start_time, self.start_time + time, self.DELAY_TIME, 2)

        self.calc(j2[0], j3[0], self.start_time, self.start_time + time, self.DELAY_TIME, 0, False)
        self.calc(j2[1], j3[1], self.start_time, self.start_time + time, self.DELAY_TIME, 1, False)
        self.calc(j2[2], j3[2], self.start_time, self.start_time + time, self.DELAY_TIME, 2, False)

        while len(self.states_done) != 6:
            self.publish_messages()

    def calc(self, x0, x1, t0, t1, t_delta, num, cords=True):
        new_value = x0 + ((x1 - x0) / (t1 - t0)) * (t_delta - t0)
        if cords:
            self.Point_pose[num] = new_value
        else:
            self.Point_rotate[num] = new_value
            
        if t_delta >= t1:
            self.states_done.append(True)
        else:
            t = Timer(self.DELAY_TIME, self.calc, [x0, x1, t0, t1, t_delta + self.DELAY_TIME, num, cords])
            t.start()


class CubicInterpolatorPoint(LinearInterpolatorPoint):
    def __init__(self):
        super(CubicInterpolatorPoint, self).__init__()

    def calc(self, x0, x1, t0, t1, t_delta, num, cords=True):
        c = (t_delta - t0) / (t1 - t0)
        a = -1 * (x1 - x0)
        b = (x1 - x0)
        new_value = (1 - c) * x0 + c * x1 + c * (1 - c) * ((1 - c) * a + c * b)

        if cords:
            self.Point_pose[num] = new_value
        else:
            self.Point_rotate[num] = new_value

        if t_delta >= t1:
            self.states_done.append(True)
        else:
            t = Timer(self.DELAY_TIME, self.calc, [x0, x1, t0, t1, t_delta + self.DELAY_TIME, num, cords])
            t.start()

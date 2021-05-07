from rclpy.clock import ROSClock
from sensor_msgs.msg import PoseStamped
from threading import Timer

class LinearInterpolatorPoint:

    def __init__(self):
        self.DELAY_TIME = 0.05
        self.start_time = 0
        self.states_done = []  # if len == 3 -> all 3 interpolations done
        self.Point_pose = [0.0 , 0.0 , 0.0]
        self.Point_rotate = [0.0 , 0.0 , 0.0]

    def publish_messages(self):
        # Joint states
        msg = PoseStamped()
        msg.header.stamp = ROSClock().now().to_msg()
        msg.name = "MarkerArray1"
        msg.position = self.Point_pose
        self.publisher.publish(msg)

    def interpolate(self, j0, j1,j2 , j3 , time, publisher):
        # j0 - initial joint positions
        # j1 - final joint positions
        self.publisher = publisher

        self.calc(j0[0], j1[0], self.start_time, self.start_time + time, self.DELAY_TIME, 0)
        self.calc(j0[1], j1[1], self.start_time, self.start_time + time, self.DELAY_TIME, 1)
        self.calc(j0[2], j1[2], self.start_time, self.start_time + time, self.DELAY_TIME, 2)

        while len(self.states_done) != 3:
            self.publish_messages()

    def calc(self, x0, x1, t0, t1, t_delta , num):
        new_value = x0 + ((x1 - x0) / (t1 - t0)) * (t_delta - t0)
        self.Point_pose[num] = new_value

        if t_delta >= t1:
            self.states_done.append(True)
        else:
            t = Timer(self.DELAY_TIME, self.calc, [x0, x1, t0, t1, t_delta + self.DELAY_TIME,num])
            t.start()


class CubicInterpolatorPoint(LinearInterpolatorPoint):
    def __init__(self):
        super(CubicInterpolatorPoint, self).__init__()

    def calc(self, x0, x1, t0, t1, t_delta,num):
        c = (t_delta - t0) / (t1 - t0)
        a = -1 * (x1 - x0)
        b = (x1 - x0)
        new_value = (1 - c) * x0 + c * x1 + c * (1 - c) * ((1 - c) * a + c * b)

        self.Point_pose[num] = new_value

        if t_delta >= t1:
            self.states_done.append(True)
        else:
            t = Timer(self.DELAY_TIME, self.calc, [x0, x1, t0, t1, t_delta + self.DELAY_TIME,num])
            t.start()

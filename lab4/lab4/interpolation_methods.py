from rclpy.clock import ROSClock
from sensor_msgs.msg import JointState
from threading import Timer
from geometry_msgs.msg import Point, Pose ,PoseStamped


class LinearInterpolator:

    def __init__(self):
        self.DELAY_TIME = 0.05
        self.start_time = 0
        self.states_done = []  # if len == 3 -> all 3 interpolations done
        self.joints_actual_positions = [0.0, 0.0, 0.0]

    def publish_messages(self):
        # Joint states
        msg = JointState()
        msg.header.stamp = ROSClock().now().to_msg()
        msg.name = ["elevator_to_rotator", "rotator_to_rotator2", "rotator2_to_arm"]
        msg.position = self.joints_actual_positions
        msg.header.frame_id = "base"
        self.publisher.publish(msg)

        msg1 = PoseStamped()
        msg1.header.stamp = ROSClock().now().to_msg()
        msg1.header.frame_id = "base2"
        msg1.pose.position.x = self.joints_actual_positions[0]
        msg1.pose.position.y = self.joints_actual_positions[1]
        msg1.pose.position.z = self.joints_actual_positions[2]
        msg1.pose.orientation.x = 0.0
        msg1.pose.orientation.y = 0.0
        msg1.pose.orientation.z = 0.0
        msg1.pose.orientation.w = 1.0
        self.publisher2.publish(msg1)

    def interpolate(self, j0, j1, time, publisher , publisher2):
        # j0 - initial joint positions
        # j1 - final joint positions
        self.publisher = publisher
        self.publisher2 = publisher2

        self.calc(j0[0], j1[0], self.start_time, self.start_time + time, self.DELAY_TIME, 0)
        self.calc(j0[1], j1[1], self.start_time, self.start_time + time, self.DELAY_TIME, 1)
        self.calc(j0[2], j1[2], self.start_time, self.start_time + time, self.DELAY_TIME, 2)

        while len(self.states_done) != 3:
            self.publish_messages()

    def calc(self, x0, x1, t0, t1, t_delta, joint_nr):
        new_value = x0 + ((x1 - x0) / (t1 - t0)) * (t_delta - t0)
        self.joints_actual_positions[joint_nr] = float(new_value)

        if t_delta >= t1:
            self.states_done.append(True)
        else:
            t = Timer(self.DELAY_TIME, self.calc, [x0, x1, t0, t1, t_delta + self.DELAY_TIME, joint_nr])
            t.start()


class CubicInterpolator(LinearInterpolator):
    def __init__(self):
        super(CubicInterpolator, self).__init__()

    def calc(self, x0, x1, t0, t1, t_delta, joint_nr):
        c = (t_delta - t0) / (t1 - t0)
        a = -1 * (x1 - x0)
        b = (x1 - x0)
        new_value = (1 - c) * x0 + c * x1 + c * (1 - c) * ((1 - c) * a + c * b)

        self.joints_actual_positions[joint_nr] = float(new_value)

        if t_delta >= t1:
            self.states_done.append(True)
        else:
            t = Timer(self.DELAY_TIME, self.calc, [x0, x1, t0, t1, t_delta + self.DELAY_TIME, joint_nr])
            t.start()

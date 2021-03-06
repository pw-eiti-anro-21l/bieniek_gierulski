import rclpy
from rclpy.clock import ROSClock
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import math
from lab5.functions import get_dh_table, forward_kin_calc


class Inv_kin(Node):

    def __init__(self):
        super().__init__('inv_kin')
        self.publisher = self.create_publisher(JointState, 'joint_states', 1)
        self.subscriber = self.create_subscription(PoseStamped, "interpolator_point", self.listener_callback, 1)
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback2, 1)
        self.previous_joints = [0, 0, 0]
        self.DH = get_dh_table()

        message = JointState()
        message.header.stamp = ROSClock().now().to_msg()
        message.name = ["elevator_to_rotator", "rotator_to_rotator2", "rotator2_to_arm"]
        message.position = [0.0, 0.0, 0.0]
        message.header.frame_id = "base"
        self.publisher.publish(message)

    def calc_inv_kin(self, actual_joints, end_point):

        try:
            d1 = end_point[2] - self.DH[0][1]

            cos_phi3 = (end_point[0] ** 2 + end_point[1] ** 2 - self.DH[3][0] ** 2 - self.DH[4][0] ** 2) / (
                    2 * self.DH[3][0] * self.DH[4][0])
            sin_phi3_1 = math.sqrt(1 - cos_phi3 ** 2)
            sin_phi3_2 = -1 * sin_phi3_1
            if cos_phi3 == 0:
                cos_phi3 = 0.000000001
            phi3_1 = math.atan2(sin_phi3_1, cos_phi3)
            phi2_1 = math.atan2(end_point[1], end_point[0]) - math.atan2(self.DH[4][0] * sin_phi3_1,
                                                                         self.DH[3][0] + self.DH[4][0] * cos_phi3)
            phi3_2 = math.atan2(sin_phi3_2, cos_phi3)
            phi2_2 = math.atan2(end_point[1], end_point[0]) - math.atan2(self.DH[4][0] * sin_phi3_2,
                                                                         self.DH[3][0] + self.DH[4][0] * cos_phi3)

            if self.angle_diff(phi2_1, actual_joints[1]) < self.angle_diff(phi2_2, actual_joints[1]):
                phi2 = self.normalize_angle(phi2_1)
                phi3 = self.normalize_angle(phi3_1)
            elif self.angle_diff(phi2_1, actual_joints[1]) >= self.angle_diff(phi2_2, actual_joints[1]):
                phi2 = self.normalize_angle(phi2_2)
                phi3 = self.normalize_angle(phi3_2)

            # Error check
            if d1 > 1 or d1 < 0:
                return None
            return [d1, phi2, phi3]

        except Exception as e:
            return None

    def normalize_angle(self, angle):
        while angle < 0:
            angle += math.radians(360)
        while angle > math.radians(360):
            angle -= math.radians(360)
        return angle

    def angle_diff(self, a1, a2):
        a1 = math.degrees(a1)
        a2 = math.degrees(a2)
        dif = float(abs(a1 - a2) % 360)
        if dif > 180:
            dif = 360 - dif
        return math.radians(dif)

    def listener_callback(self, msg):
        result = self.calc_inv_kin(self.previous_joints,
                                   [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        if result is not None:
            message = JointState()
            message.header.stamp = ROSClock().now().to_msg()
            message.name = ["elevator_to_rotator", "rotator_to_rotator2", "rotator2_to_arm"]
            message.position = result
            message.header.frame_id = "base"
            self.publisher.publish(message)
            self.previous_joints = result
        else:
            self.get_logger().error("Can't reach given position")

    def listener_callback2(self, msg):
        self.previous_joints[0] = msg.position[0]
        self.previous_joints[1] = msg.position[1]
        self.previous_joints[2] = msg.position[2]


def main(args=None):
    rclpy.init(args=args)

    oint_pub = Inv_kin()

    rclpy.spin(oint_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    oint_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

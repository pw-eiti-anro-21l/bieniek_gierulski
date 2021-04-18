import rclpy
from rclpy.node import Node
import numpy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Pose, PoseStamped
from PyKDL import *
from lab3_essential.functions import *
from rclpy.clock import ROSClock
from geometry_msgs.msg import Quaternion


class KDL_fk(Node):

    def __init__(self):
        super().__init__('kdl_fk')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback, 1)
        self.publisher = self.create_publisher(PoseStamped, 'kdl_fk', 1)
        self.params = load_yaml()
        self.create_chain()

    def create_chain(self):
        j1_rpy = self.convert_string(self.params["joints"]["joint1"]["rpy"])
        j1_xyz = self.convert_string(self.params["joints"]["joint1"]["xyz"])
        j3_rpy = self.convert_string(self.params["joints"]["joint3"]["rpy"])
        j3_xyz = self.convert_string(self.params["joints"]["joint3"]["xyz"])
        j4_rpy = self.convert_string(self.params["joints"]["joint4"]["rpy"])
        j4_xyz = self.convert_string(self.params["joints"]["joint4"]["xyz"])

        self.chain = Chain()
        j0 = Joint(Joint.TransZ)
        f0 = Frame(Rotation.RPY(j1_rpy[0], j1_rpy[1], j1_rpy[2]), Vector(j1_xyz[0], j1_xyz[1], j1_xyz[2]))
        s0 = Segment(j0, f0)
        self.chain.addSegment(s0)
        j1 = Joint(Joint.TransZ)
        f1 = Frame(Rotation.RPY(j3_rpy[0], j3_rpy[1], j3_rpy[2]), Vector(0, j3_xyz[1], j3_xyz[2]))
        s1 = Segment(j1, f1)
        self.chain.addSegment(s1)
        j3 = Joint(Joint.RotZ)
        f3 = Frame(Rotation.RPY(j4_rpy[0], j4_rpy[1], j4_rpy[2]), Vector(1, j4_xyz[1], j4_xyz[2]))
        s3 = Segment(j3, f3)
        self.chain.addSegment(s3)
        j4 = Joint(Joint.RotZ)
        f4 = Frame(Rotation.RPY(0, 0, 0), Vector(0, 0, 0))
        s4 = Segment(j4, f4)
        self.chain.addSegment(s4)

        # manipulator
        j5 = Joint(Joint.TransX)
        f5 = Frame(Rotation.RPY(0, 0, 0), Vector(0, 0, 0))
        s5 = Segment(j5, f5)
        self.chain.addSegment(s5)

    def listener_callback(self, msg):
        jointAngles = JntArray(5)
        jointAngles[0] = 0.0
        jointAngles[1] = msg.position[0]
        jointAngles[2] = msg.position[1]
        jointAngles[3] = msg.position[2]
        jointAngles[4] = float(self.params["links"]["link4"]["length"])
        fk = ChainFkSolverPos_recursive(self.chain)
        finalFrame = Frame()
        fk.JntToCart(jointAngles, finalFrame)
        position = finalFrame.p
        matrix = finalFrame.M

        point = Point()
        point.x = position[0]
        point.y = position[1]
        point.z = position[2]
        quaternion = matrix.GetQuaternion()
        quaternion = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
        pose = Pose()
        pose.position = point
        pose.orientation = quaternion
        pose_st = PoseStamped()
        pose_st.pose = pose
        pose_st.header.stamp = ROSClock().now().to_msg()
        pose_st.header.frame_id = "base"
        self.publisher.publish(pose_st)

    def convert_string(self, str):
        new = []
        for nmbr in str.split():
            new.append(float(nmbr))
        return new


def main(args=None):
    rclpy.init(args=args)
    kdl_fk = KDL_fk()
    rclpy.spin(kdl_fk)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    kdl_fk.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
import numpy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Pose, PoseStamped
from math import cos, sin
from lab3_essential.functions import *
from rclpy.clock import ROSClock
from visualization_msgs.msg import MarkerArray, Marker
from lab4.functions import *

class Draw_point(Node):

    def __init__(self):
        super().__init__('draw_point')
        self.subscription = self.create_subscription(
            PoseStamped,
            'oint_pub',
            self.listener_callback, 1)
        self.publisher2 = self.create_publisher(MarkerArray, "trajectory_point", 1)
        self.markers = MarkerArray()

    def listener_callback(self, msg):
        print("heared")
        pose_st = PoseStamped()
        pose_st.pose = msg.pose
        pose_st.header.stamp = ROSClock().now().to_msg()
        pose_st.header.frame_id = "base"

        marker = Marker()
        marker.header.stamp = ROSClock().now().to_msg()
        marker.header.frame_id = "/base"
        marker.type = 2
        marker.pose = pose_st.pose
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
        print(self.markers.markers)




def main(args=None):
    rclpy.init(args=args)

    draw = Draw_point()

    rclpy.spin(draw)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    draw.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

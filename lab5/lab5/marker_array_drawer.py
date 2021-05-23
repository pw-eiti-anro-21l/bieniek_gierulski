import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from lab5.functions import *
from rclpy.clock import ROSClock
from visualization_msgs.msg import MarkerArray, Marker

class Draw_point(Node):

    def __init__(self):
        super().__init__('draw_point')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback, 1)
        self.publisher = self.create_publisher(MarkerArray, "trajectory", 1)
        self.markers = MarkerArray()
        self.DH = get_dh_table()

    def listener_callback(self, msg):
        position, matrix = forward_kin_calc(self.DH, msg.position)
        pose = Pose()
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]

        marker = Marker()
        marker.header.stamp = ROSClock().now().to_msg()
        marker.header.frame_id = "base"
        marker.type = 2
        marker.pose = pose
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
        self.publisher.publish(self.markers)




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
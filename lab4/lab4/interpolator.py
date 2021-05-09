import rclpy
from rclpy.clock import ROSClock
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Pose ,PoseStamped
from lab4_service.srv import Interpolation
from lab4.interpolation_methods import LinearInterpolator, CubicInterpolator
from rclpy.node import Node
from rclpy.qos import QoSProfile



class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Interpolation, 'Interpolation', self.interpolation_callback)
        self.publisher = self.create_publisher(JointState, 'joint_states', QoSProfile(depth = 10))
        self.publisher2 = self.create_publisher(PoseStamped, 'plot_info', QoSProfile(depth=10))
       # self.subscription = self.create_subscription(
       #     JointState,
       #     'joint_states',
       #     self.listener_callback, 1)
        self.joint_positions = [0.0, 0.0, 0.0]

        # publish 1 time
        msg1 = JointState()
        msg1.header.stamp = ROSClock().now().to_msg()
        msg1.name = ["elevator_to_rotator", "rotator_to_rotator2", "rotator2_to_arm"]
        msg1.position = self.joint_positions
        msg1.header.frame_id = "base"
        self.publisher.publish(msg1)

        msg = PoseStamped()
        msg.header.stamp = ROSClock().now().to_msg()
        msg.header.frame_id = "base2"
        msg.pose.position.x = 0.0
        msg.pose.position.z = 0.0
        msg.pose.position.y = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        self.publisher2.publish(msg)


    #def listener_callback(self, msg):
    #    self.joint_positions[0] = msg.position[0]
    #    self.joint_positions[1] = msg.position[1]
    #    self.joint_positions[2] = msg.position[2]

    def interpolation_callback(self, request, response):
        j1 = [request.joint1_pos, request.joint2_pos, request.joint3_pos]
        time = request.time
        if request.method == "linear":
            interpolator = LinearInterpolator()
            interpolator.interpolate(self.joint_positions, j1, time, self.publisher,self.publisher2)
        elif request.method == "cubic":
            interpolator = CubicInterpolator()
            interpolator.interpolate(self.joint_positions, j1, time, self.publisher,self.publisher2)

        response.result = "Done"
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

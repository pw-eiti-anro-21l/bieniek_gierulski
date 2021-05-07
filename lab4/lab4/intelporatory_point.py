import rclpy
from rclpy.clock import ROSClock
from sensor_msgs.msg import JointState
from lab4_service.srv import Interpolation
from lab4.interpolation_methods_point import LinearInterpolatorPoint, CubicInterpolatorPoint
from rclpy.node import Node


class MinimalServicePoint(Node):

    def __init__(self):
        super().__init__('minimal_service_oint')
        self.srv = self.create_service(Interpolation, 'Interpolation', self.interpolation_callback)
        self.publisher = self.create_publisher(JointState, 'joint_states', 1)
        self.joint_positions = [0.0, 0.0, 0.0]

    def listener_callback(self, msg):
        self.joint_positions[0] = msg.position[0]
        self.joint_positions[1] = msg.position[1]
        self.joint_positions[2] = msg.position[2]

    def interpolation_callback(self, request, response):
        j1 = [request.joint1_pos]
        time = request.time
        if request.method == "linear":
            interpolator = LinearInterpolatorPoint()
            interpolator.interpolate(self.joint_positions, j1, time, self.publisher)
        elif request.method == "cubic":
            interpolator = CubicInterpolatorPoint()
            interpolator.interpolate(self.joint_positions, j1, time, self.publisher)

        response.result = "Done"
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalServicePoint()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

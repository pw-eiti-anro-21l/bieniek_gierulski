import sys
from lab4_service.srv import InterpolationPoint
import rclpy
from rclpy.node import Node
from lab5.draw_figure import Draw_figure



class MinimalClientAsyncPoint(Node):

    def __init__(self):
        super().__init__('oint')
        self.cli = self.create_client(InterpolationPoint, 'InterpolationPoint')
        while not self.cli.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('service not available, waiting again...')
        self.req = InterpolationPoint.Request()
        self.METHODS = ["linear", "cubic","rectangle" , "elipse"]
        self.methodd = None
        self.ab = None

    def send_request(self):
        # check data
        # check data
        self.req.x_pos = float(sys.argv[1])
        self.req.y_pos = float(sys.argv[2])
        self.req.z_pos = float(sys.argv[3])
        self.req.roll = float(sys.argv[4])
        self.req.pitch = float(sys.argv[5])
        self.req.yaw = float(sys.argv[6])
        self.req.time = float(sys.argv[7])
        self.req.method = str(sys.argv[8])
        self.methodd = str(sys.argv[8])
        self.ab = [float(sys.argv[1]) , float(sys.argv[2])]
        if self.req.time <= 0:
            raise Exception("Invalid time")
        if self.req.method not in self.METHODS:
            raise Exception("Invalid method")
        #if self.req.joint1_pos > 1 or self.req.joint1_pos < 0:
        #    raise Exception("Invalid joint1 position")
        #if self.req.joint2_pos > 2 * math.pi or self.req.joint2_pos < 0:
        #    raise Exception("Invalid joint2 position")
        #if self.req.joint3_pos > math.pi or self.req.joint3_pos < -1 * math.pi:
        #    raise Exception("Invalid joint3 position")

        # send
        self.future = self.cli.call_async(self.req)

    def get_method(self):
        return self.methodd

    def get_ab(self):
        return [self.ab]

def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsyncPoint()
    minimal_client.send_request()
    ab = minimal_client.get_ab()
    a = ab[0]
    b = ab[1]

    draw_figure = Draw_figure
    while rclpy.ok():
        if (minimal_client.get_method() == "elipse"):
            while True:
                current_trucked = draw_figure.calc_elipse(a,b)
                x = current_trucked[0]
                y = current_trucked[1]
                # aktualiziacja x,y w pliku
                rclpy.spin_once(minimal_client)
                response = minimal_client.future.result()
        elif (minimal_client.get_method() == "rectangle"):
            point = 0
            edges = draw_figure.calc_corners(a,b)
            while True:
                current_trucked = edges[point]
                x = current_trucked[0]
                y = current_trucked[1]
                # aktualiziacja x,y w pliku
                rclpy.spin_once(minimal_client)

                point = point+1
                if (point == 4):
                   point = 0
                response = minimal_client.future.result()
        else:
            rclpy.spin_once(minimal_client)
            if minimal_client.future.done():
                try:
                    response = minimal_client.future.result()
                except Exception as e:
                    minimal_client.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    minimal_client.get_logger().info(
                        'Result: %s' %
                        response.result)
                break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

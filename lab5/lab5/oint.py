import sys
from lab4_service.srv import InterpolationPoint
import rclpy
from rclpy.node import Node
from lab5.shapes_drawer import RectangleDrawer, ElipseDrawer




class MinimalClientAsyncPoint(Node):

    def __init__(self):
        super().__init__('oint')
        self.cli = self.create_client(InterpolationPoint, 'InterpolationPoint')
        while not self.cli.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('service not available, waiting again...')
        self.req = InterpolationPoint.Request()
        self.METHODS = ["linear", "cubic"]
        self.drawer = None


    def send_request(self, x, y, z, roll, pitch, yaw, time, method):
        # check data
        # check data

        self.req.x_pos = x
        self.req.y_pos = y
        self.req.z_pos = z
        self.req.roll = roll
        self.req.pitch = pitch
        self.req.yaw = yaw
        self.req.time = time
        self.req.method = method

        if self.req.time <= 0:
            raise Exception("Invalid time")
        if self.req.method not in self.METHODS:
            raise Exception("Invalid method")
        self.future = self.cli.call_async(self.req)

    def get_method(self):
        return self.methodd

    def get_ab(self):
        return [self.ab]

def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsyncPoint()


    if sys.argv[1] != "rectangle" and sys.argv[1] != "elipse":
        try:
            minimal_client.send_request(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]),
                                        float(sys.argv[5]), float(sys.argv[6]), float(sys.argv[7]), str(sys.argv[8]))
        except:
            print("Invalid arguments")
            sys.exit(0)
        while rclpy.ok():

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
    elif sys.argv[1] == "rectangle":
        minimal_client.drawer = RectangleDrawer(sys.argv[2], sys.argv[3])
        while True:
            try:
                point = minimal_client.drawer.get_next_point()
                minimal_client.send_request(point[0], point[1], point[2], 0.0, 0.0, 0.0, 8.0, "linear")
            except Exception as e:
                print(e)
                sys.exit(0)
            while rclpy.ok():
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
    elif sys.argv[1] == "elipse":
        minimal_client.drawer = ElipseDrawer(sys.argv[2], sys.argv[3])
        first = True
        while True:
            try:
                time = 0.1 if not first else 5.0
                first = False
                point = minimal_client.drawer.get_next_point()
                minimal_client.send_request(point[0], point[1], point[2], 0.0, 0.0, 0.0, time, "linear")
            except:
                print("Invalid arguments")
                sys.exit(0)
            while rclpy.ok():
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

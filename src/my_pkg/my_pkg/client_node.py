import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts

import sys

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_clinet_async')
        self.cli = self.create_client(AddTwoInts,'add_two_ints')
    
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not availabe , waiting')
        self.req = AddTwoInts.Request()
        

    def send_request(self,a,b):
        self.req.a  = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self,self.future)

        return self.future.result()
    
def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()

    a = int(sys.argv[1])
    b = int(sys.argv[2])

    response = minimal_client.send_request(a,b)
    minimal_client.get_logger().info(f"Result of add_two_ints: {response.sum}")

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

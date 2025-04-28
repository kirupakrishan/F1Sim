import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10) # Create a publisher with name 'topic'
        timer_period = 0.5
        self.timer = self.create_timer(timer_period,self.timer_callback)  # Create a timer that calls timer_callback every 0.5 seconds
        self.i=0

    def timer_callback(self):
        msg = String()
        msg.data = f"Hello Naan Dhan da kingu : {self.i}"
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    print('Hi from my_pkg.')
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

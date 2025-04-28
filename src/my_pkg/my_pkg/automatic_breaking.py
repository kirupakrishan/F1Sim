import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class AutomaticEmergencyBracking(Node):

    def __init__(self):
        super().__init__('automatic_emergency_bracking')
        self.scan_sub = self.create_subscription(
            LaserScan,  #Msg type
            'scan',     #topic name
            self.listener_callback, # callback function
            10   # queue size
        )
        self.odom_sub = self.create_subscription(
            Odometry,  #Msg type
            'odom',     #topic name
            self.odom_callback, # callback function
            10   # queue size
        )
        self.publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        self.lin_vel = 1

    def odom_callback(self, msg):
        self.lin_vel = msg.twist.twist.linear.x

    def listener_callback(self, msg):
        robot_move = Twist()
        range_msg = np.array(msg.ranges)
        angles = np.arange(len(range_msg)) * msg.angle_increment + msg.angle_min

        cone_mask = (angles > -0.349) & (angles < 0.349)

        # Filter ranges and angles within that cone
        ranges_cone = range_msg[cone_mask]
        angles_cone = angles[cone_mask]

        # Ignore invalid or too-close ranges
        valid_mask = np.isfinite(ranges_cone) & (ranges_cone > 0.05)
        ranges_cone = ranges_cone[valid_mask]
        angles_cone = angles_cone[valid_mask]
        
        range_rate = self.lin_vel*np.cos(angles_cone)
        range_rate = np.where(range_rate > 0 ,range_rate,1e-6)
        ttc = ranges_cone/ range_rate
        ttc = np.where(ttc>=0, ttc, np.inf)
        self.get_logger().info("min_range " + str(np.min(range_msg)))
        self.get_logger().info("min_range_rate " + str(np.min(range_rate)))
        self.get_logger().info("ttc bool: " + str(np.any(ttc < 1))+" "+ str(np.min(ttc)))
        self.get_logger().info("ttc: " + str(ttc))
        min_ttc = np.min(ttc)
        if min_ttc< 1:
            self.get_logger().info("Obstacle detected! Stopping the robot.")
            robot_move.linear.x = 0.0
            robot_move.angular.z = 0.0
        # elif min_ttc >= 1 and min_ttc < 1.5:
        #     self.get_logger().info("Obstacle detected! Slowing down the robot.")
        #     robot_move.linear.x = 0.5
        #     robot_move.angular.z = 0.0
        # elif min_ttc >= 1.5 and min_ttc < 2:
        #     self.get_logger().info("Obstacle detected! Slowing down the robot.")
        #     robot_move.linear.x = 1.0
        #     robot_move.angular.z = 0.0
        else:
            self.get_logger().info("No obstacle detected.")
            robot_move.linear.x = 1.0
            robot_move.angular.z = 0.0
        
        self.publisher.publish(robot_move)

    
def main(args=None):
    rclpy.init(args=args)
    aeb = AutomaticEmergencyBracking()
    rclpy.spin(aeb)

    aeb.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Mover(Node):
    """
    A very simple Roamer implementation for LIMO.
    It simply goes straight until any obstacle is within
    2 m distance and then just simply turns left.
    A purely reactive approach.
    """
    def __init__(self):
        """
        On construction of the object, create a Subscriber
        to listen to laser scans and a Publisher to control
        the robot
        """
        # super().__init__('tf_listener')
        super().__init__('robot_nav')
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.subscriber = self.create_subscription(LaserScan, "/scan", self.laserscan_callback, 10)
        
        # message to show the node is active
        self.get_logger().info (f"Bot in motion")
    
    def laserscan_callback(self, data):
        """
        Callback called any time a new laser scan become available
        """
        min_dist = min(data.ranges[int(len(data.ranges)/2) -30 : int(len(data.ranges)/2) +30])
        left = data.ranges[int(len(data.ranges)/2) -30]
        right = data.ranges[int(len(data.ranges)/2) +30]

        # print("Min: ", min_dist)
        t = Twist()
        if min_dist < 0.4: #changed from 2m to maybe 1.75m?not sure of the conversion
            t.angular.z = -0.2 #turns right instead because my robot starts in a different position?
        else:
            t.linear.x = 0.3 # speed is much slower now
        self.publisher.publish(t)


def main(args=None):
    rclpy.init(args=args)
    mover = Mover()
    rclpy.spin(mover)

    mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
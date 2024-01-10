import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
import numpy as np

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped, '/limo/object_location',
            self.posestamped_callback, 10
        )
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.coordinates = []

    # convert to tf
    def get_tf_transform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return transform
        except Exception as e:
            self.get_logger().warning(f"Failed to lookup transform: {str(e)}")
            return None

    def posestamped_callback(self, object_location):
        try:
            transform = self.get_tf_transform('odom', 'depth_link') 
            pothole_relative_to_odom = do_transform_pose(object_location.pose, transform)
        except Exception as e:
            raise ValueError(e)
        new_x = pothole_relative_to_odom.position.x
        new_y = pothole_relative_to_odom.position.y
        if self.threshold_check(new_x, new_y):
            return
        self.coordinates.append((new_x, new_y))
        print("Count : ", len(self.coordinates)) # Number of potholes as length of the array

    # change variable names and learn em
    def threshold_check(self, new_x, new_y):
        threshold = 0.15 # min pythagoras distance between 2 pothole coords. Still working on the appropriate value 
        for coord in self.coordinates:
            if self.calculate_distance(coord[0], coord[1], new_x, new_y) < threshold:
                return True
        return False

    def calculate_distance(self, x1, y1, x2, y2):
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    

    # print out the coordinates in the odom frame
           
        
    # # Part of the pothole counter...
            # pothole_relative_to_odom = PoseStamped()
            # pothole_relative_to_odom.header.frame_id = 'odom' # frame position is odom, the world
            # pothole_relative_to_odom.pose = p_camera
            # self.object_location_pub.publish(pothole_relative_to_odom)

def main(args=None):
    rclpy.init(args=args)
    pose_subscriber = PoseSubscriber()
    rclpy.spin(pose_subscriber)
    pose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import numpy as np

class PotholeCounter(Node):
    def __init__(self):
        super().__init__('object_counter')

        # Subscribe to the marker value called in the detector node
        self.marker_pub = self.create_subscription(Marker, '/marker', self.marker_callback,  1)

        # Message to the logger to show the node is active
        self.get_logger().info (f"Reporter is on")

        # intialize the array to hold the pothole coordinates.
        self.coordinates = []

    def marker_callback(self, msg):
        try:
            new_x = msg.pose.position.x
            new_y = msg.pose.position.y
        except Exception as e:
            raise ValueError(e)
        
        if self.threshold_check(new_x, new_y):
                return

        self.coordinates.append((new_x, new_y))
        # Prints the Pothole count value to the log screen
        self.get_logger().info (f"Pothole Count : {len(self.coordinates)}")
        # Prints the Pothole count value to the log screen
        self.get_logger().info (f"Pothole Location : {(new_x, new_y)}")

    def threshold_check(self, new_x, new_y):
        threshold = 0.095 # min distance between 2 pothole coordinates
        for coord in self.coordinates:
            if self.calculate_distance(coord[0], coord[1], new_x, new_y) < threshold:
                return True
        return False

    def calculate_distance(self, x1, y1, x2, y2):
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    
    
def main(args=None):
    rclpy.init(args=args)
    pothole_counter = PotholeCounter()
    try:
        rclpy.spin(pothole_counter)
    except KeyboardInterrupt:
        # Save the report to a text file
        filename = 'report.txt'
        with open(filename, 'w') as file:
            file.write(f'Number of potholes found : {len(pothole_counter.coordinates)}\n\n')
            for count, (x, y) in enumerate(pothole_counter.coordinates, start=1):
                file.write(f'Pothole {count}. {x}, {y}\n')
            print(f'Report save to {filename}.')

    pothole_counter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
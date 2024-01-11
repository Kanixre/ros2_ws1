from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = "robot_controller_1",
            executable = "robot_nav",
            name = 'robot_nav_node',
            output = 'log' 
        ),
        Node(
            package = "robot_controller_1",
            executable = "object_detector",
            name = 'object_detector_node',
            output = 'log' 
        ),
        Node(
            package = "robot_controller_1",
            executable = "object_counter",
            name = 'object_counter_node',
            output = 'log' 
        )
    ])
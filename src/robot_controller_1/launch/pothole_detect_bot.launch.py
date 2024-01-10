from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = "robot_controller1",
            executable = "robot_nav",
            name = 'mover_node'
        ),
        Node(
            package = "robot_controller1",
            executable = "object_detector",
            name = 'detector_node'
        ),
        Node(
            package = "robot_controller1",
            executable = "object_counter",
            name = 'counter_node',
            output = 'log' 
        )
    ])
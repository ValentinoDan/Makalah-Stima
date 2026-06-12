from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="vis_astar",
            executable="planner_node",
            output="screen"
        )
    ])
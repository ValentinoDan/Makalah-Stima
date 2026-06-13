import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("vis_astar")

    world_file = os.path.join(pkg, "worlds", "world.world")
    urdf_file = os.path.join(pkg, "urdf",   "robot.urdf")
    map_file = os.path.join(pkg, "maps",   "map.yaml")
    rviz_file = os.path.join(pkg, "config", "vis_astar.rviz")

    with open(urdf_file, "r") as f:
        robot_description = f.read()

    use_rviz = LaunchConfiguration("use_rviz", default="true")

    return LaunchDescription([

        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz2 for visualization",
        ),

        # ExecuteProcess(
        #     cmd=[
        #         "gazebo", "--verbose", world_file,
        #         "-s", "libgazebo_ros_init.so",
        #         "-s", "libgazebo_ros_factory.so",
        #     ],
        #     output="screen",
        # ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        ),

        # Node(
        #     package="gazebo_ros",
        #     executable="spawn_entity.py",
        #     arguments=[
        #         "-topic", "robot_description",
        #         "-entity", "diff_robot",
        #         "-x", "-8.0",
        #         "-y", "-8.0",
        #         "-z",  "0.1",
        #     ],
        #     output="screen",
        # ),

        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{
                "yaml_filename": map_file,
                "frame_id": "map",
                "use_sim_time": True,
            }],
        ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map",
            output="screen",
            parameters=[{
                "autostart": True,
                "node_names": ["map_server"],
                "use_sim_time": True,
            }],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_tf",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            output="screen",
        ),

        Node(
            package="vis_astar",
            executable="planner_node",
            name="planner_node",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),

        Node(
            package="vis_astar",
            executable="path_follower_node",
            name="path_follower_node",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),

        Node(
            package='vis_astar',
            executable='odom_node',
            output='screen',
            parameters=[{
                'publish_rate': 20.0,
                'initial_x':    0.0,
                'initial_y':    0.0,
                'initial_yaw':  0.0,
            }],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_file],
            output="screen",
            condition=IfCondition(use_rviz),
            parameters=[{"use_sim_time": True}],
        ),
    ])
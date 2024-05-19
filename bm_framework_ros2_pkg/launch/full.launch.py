from launch import LaunchDescription
from launch_ros.actions import Node

package_name = 'bm_framework_ros2_pkg'

def generate_launch_description():
    launch_description = LaunchDescription([
        Node(
            package="bm_framework_ros2_pkg",
            executable="bm_limb_readings"
        ),
        Node(
            package="bm_framework_ros2_pkg",
            executable="bm_relay"
        ),
        Node(
            package="bm_framework_ros2_pkg",
            executable="bm_server"
        ),
        Node(
            package="bm_framework_ros2_pkg",
            executable="bm_app"
        ),
        # Node(
        #     package="bm_framework_ros2_pkg",
        #     executable="bm_app_orientations"
        # ),
    ])
    return launch_description

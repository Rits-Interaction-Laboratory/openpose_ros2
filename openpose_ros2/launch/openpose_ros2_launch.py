from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package="openpose_ros2",
            executable="openpose_ros2",
            name="openpose_node",
            output="screen",
            respawn="true",
            parameters=[
                {
                    "is_debug_mode": False,
                    "openpose_root": "/openpose",
                    "is_image_compressed": True,
                    "image_node": "/rs/color/compressed"
                }
            ]
        )
    ])
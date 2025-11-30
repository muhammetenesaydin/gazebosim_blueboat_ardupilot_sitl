from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='blueboat_autonomy',
            executable='autonomy_node',
            name='blueboat_autonomy',
            output='screen',
            parameters=[
                # Add parameters here if needed
            ]
        )
    ])

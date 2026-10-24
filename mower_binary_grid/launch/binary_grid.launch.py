from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mower_binary_grid',
            executable='binary_grid_node',
            name='binary_grid_node',
            output='screen',
            parameters=[{
                # TODO: 장착 완료 후 네 값만 실측값으로 변경한다.
                'camera_forward_m': 0.0,
                'camera_left_m': 0.0,
                'camera_height_m': 0.60,
                'camera_pitch_down_deg': 0.0,
                'resolution_m': 0.20,
                'forward_max_m': 6.0,
                'lateral_min_m': -3.0,
                'lateral_max_m': 3.0,
                'depth_max_m': 6.0,
            }],
        ),
    ])


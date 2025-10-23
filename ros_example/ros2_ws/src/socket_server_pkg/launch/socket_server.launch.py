from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='socket_server_pkg',
            executable='socket_server_node',
            name='socket_server_node',
            output='screen',
            parameters=[{
                'port': 5000,
                'topic_in': '/socket_in',
                'topic_out': '/socket_out',
                'backlog': 8,
                'tcp_nodelay': True,
                'recv_buffer': 1048576,
                'send_buffer': 1048576,
            }]
        )
    ])


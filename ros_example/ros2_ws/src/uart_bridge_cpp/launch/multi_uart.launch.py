from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    baud      = LaunchConfiguration('baud')
    newline   = LaunchConfiguration('newline')
    poll_ms   = LaunchConfiguration('poll_ms')
    rx_chunk  = LaunchConfiguration('rx_chunk')
    log_bytes = LaunchConfiguration('log_bytes')
    log_lines = LaunchConfiguration('log_lines')

    return LaunchDescription([
        DeclareLaunchArgument('baud',      default_value='115200'),
        DeclareLaunchArgument('newline',   default_value='\n'),
        DeclareLaunchArgument('poll_ms',   default_value='50'),
        DeclareLaunchArgument('rx_chunk',  default_value='512'),
        DeclareLaunchArgument('log_bytes', default_value='false'),
        DeclareLaunchArgument('log_lines', default_value='true'),

        GroupAction(actions=[  # /dev/serial0 기본 노드
            Node(
                package='uart_bridge_cpp',
                executable='uart_bridge_node',
                name='uart_bridge_serial0',
                output='screen',
                parameters=[{
                    'port': '/dev/serial0',
                    'baud': baud,
                    'newline': newline,
                    'rx_topic': '/uart/rx',
                    'tx_topic': '/uart/tx',
                    'poll_ms': poll_ms,
                    'rx_chunk': rx_chunk,
                    'log_bytes': log_bytes,
                    'log_lines': log_lines,
                }],
            )
        ]),

        GroupAction(actions=[  # /dev/ttyAMA1 전용 노드
            Node(
                package='uart_bridge_cpp',
                executable='uart_bridge_ama1_node',
                name='uart_bridge_ama1',
                output='screen',
                parameters=[{
                    'baud': baud,
                    'newline': newline,
                    'rx_topic': '/uart/ama1/rx',
                    'tx_topic': '/uart/ama1/tx',
                    'poll_ms': poll_ms,
                    'rx_chunk': rx_chunk,
                    'log_bytes': log_bytes,
                    'log_lines': log_lines,
                }],
            )
        ]),
    ])

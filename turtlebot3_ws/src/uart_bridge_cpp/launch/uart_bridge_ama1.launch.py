from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
        DeclareLaunchArgument('newline',   default_value='\n'),  # 또는 '\r\n'
        DeclareLaunchArgument('poll_ms',   default_value='50'),
        DeclareLaunchArgument('rx_chunk',  default_value='512'),
        DeclareLaunchArgument('log_bytes', default_value='false'),
        DeclareLaunchArgument('log_lines', default_value='true'),

        Node(
            package='uart_bridge_cpp',             # 패키지명 확인
            executable='uart_bridge_ama1_node',    # CMake에 추가했던 실행파일명
            name='uart_bridge_ama1',
            output='screen',
            parameters=[{
                # 포트는 소스에서 /dev/ttyAMA1 로 하드코딩됨
                'baud': baud,
                'newline': newline,
                'rx_topic': '/uart/ama1/rx',
                'tx_topic': '/uart/ama1/tx',
                'poll_ms': poll_ms,
                'rx_chunk': rx_chunk,
                'log_bytes': log_bytes,
                'log_lines': log_lines,
            }],
        ),
    ])

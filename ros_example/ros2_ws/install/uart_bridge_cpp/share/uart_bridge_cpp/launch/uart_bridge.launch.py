from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uart_bridge_cpp',
            executable='uart_bridge_node',
            name='uart_bridge_node',
            output='screen',
            parameters=[{
                'port': '/dev/serial0',   # ← 여기만 serial0 으로
                'baud': 115200,
                'newline': '\n',          # '\n' 또는 '\r\n'
                'rx_topic': '/uart/rx',
                'tx_topic': '/uart/tx',
                'poll_ms': 50,            # select 타임아웃(ms)
                'rx_chunk': 512,          # 1회 최대 읽기 바이트
                'log_bytes': False        # True면 수신 raw 바이트 로그
            }]
        )
    ])


# launch/multi_uart.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 공통 파라미터 (필요 시 런치 인자로 조정)
    baud      = LaunchConfiguration('baud')
    newline   = LaunchConfiguration('newline')
    poll_ms   = LaunchConfiguration('poll_ms')
    rx_chunk  = LaunchConfiguration('rx_chunk')
    log_bytes = LaunchConfiguration('log_bytes')
    log_lines = LaunchConfiguration('log_lines')

    return LaunchDescription([
        # 공통 인자 선언(원하면 실행 시 덮어쓰기 가능)
        DeclareLaunchArgument('baud',      default_value='115200'),
        DeclareLaunchArgument('newline',   default_value='\n'),   # 또는 '\r\n'
        DeclareLaunchArgument('poll_ms',   default_value='50'),
        DeclareLaunchArgument('rx_chunk',  default_value='512'),
        DeclareLaunchArgument('log_bytes', default_value='false'),
        DeclareLaunchArgument('log_lines', default_value='true'),

        # --- 노드 1: /dev/serial0 브리지 (기존 일반 노드) ---
        GroupAction(actions=[
            Node(
                package='uart_bridge_cpp',              # ← 패키지명 확인
                executable='uart_bridge_node',          # 기존 실행파일
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

        # --- 노드 2: /dev/ttyAMA1 브리지 (고정 전용 새 노드) ---
        GroupAction(actions=[
            Node(
                package='uart_bridge_cpp',              # ← 패키지명 확인
                executable='uart_bridge_ama1_node',     # 새로 추가한 실행파일
                name='uart_bridge_ama1',
                output='screen',
                parameters=[{
                    # 포트는 소스에서 /dev/ttyAMA1 고정
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


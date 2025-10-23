from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # LaunchConfigurations
    baud      = LaunchConfiguration('baud')
    newline   = LaunchConfiguration('newline')   # 실제 개행 문자 전달
    poll_ms   = LaunchConfiguration('poll_ms')
    rx_chunk  = LaunchConfiguration('rx_chunk')
    log_bytes = LaunchConfiguration('log_bytes')
    log_lines = LaunchConfiguration('log_lines')

    return LaunchDescription([
        # 기본값 선언 (newline은 실제 개행 문자)
        DeclareLaunchArgument('baud',      default_value='115200'),
        DeclareLaunchArgument('newline',   default_value='\n'),
        DeclareLaunchArgument('poll_ms',   default_value='50'),
        DeclareLaunchArgument('rx_chunk',  default_value='512'),
        DeclareLaunchArgument('log_bytes', default_value='false'),
        DeclareLaunchArgument('log_lines', default_value='true'),

        # /dev/serial0 기본 노드
        GroupAction(actions=[
            Node(
                package='uart_bridge_cpp',
                executable='uart_bridge_node',
                name='uart_bridge_serial0',
                output='screen',
                parameters=[{
                    'port': '/dev/serial0',
                    'baud':      ParameterValue(baud, value_type=int),
                    'newline':   ParameterValue(newline, value_type=str),
                    'rx_topic':  '/uart/rx',
                    'tx_topic':  '/uart/tx',
                    'poll_ms':   ParameterValue(poll_ms, value_type=int),
                    'rx_chunk':  ParameterValue(rx_chunk, value_type=int),
                    'log_bytes': ParameterValue(log_bytes, value_type=bool),
                    'log_lines': ParameterValue(log_lines, value_type=bool),
                }],
            )
        ]),

        # /dev/ttyAMA1 전용 노드
        GroupAction(actions=[
            Node(
                package='uart_bridge_cpp',
                executable='uart_bridge_ama1_node',
                name='uart_bridge_ama1',
                output='screen',
                parameters=[{
                    # 코드 기본값이 AMA1이지만, 명시해두면 더 안전
                    'port': '/dev/ttyAMA1',
                    'baud':      ParameterValue(baud, value_type=int),
                    'newline':   ParameterValue(newline, value_type=str),
                    # ★ 여기 토픽은 네가 쓰는 이름과 일치시키자.
                    #   지금처럼 /stm/*로 쓸 거면 퍼블/에코도 /stm/*로!
                    'rx_topic':  '/stm/rx',
                    'tx_topic':  '/stm/tx',
                    'poll_ms':   ParameterValue(poll_ms, value_type=int),
                    'rx_chunk':  ParameterValue(rx_chunk, value_type=int),
                    'log_bytes': ParameterValue(log_bytes, value_type=bool),
                    'log_lines': ParameterValue(log_lines, value_type=bool),
                }],
            )
        ]),
    ])


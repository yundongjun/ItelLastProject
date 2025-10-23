from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rescue_orchestrator",
            executable="rescue_orchestrator_node",
            name="rescue_orchestrator",
            output="screen",
            parameters=[{
                "uart_cmd_topic": "/uart/rx",
                "forklift_up_service": "/turtlebot3_node/forklift_up",
                "forklift_down_service": "/turtlebot3_node/forklift_down",
                "rotate_deg": 90.0,          # LEFT=-90, RIGHT=+90
                "home_x": 0.0,
                "home_y": 0.0,
                "home_yaw": 0.0,            # degrees
                "nav_timeout_ms": 20000,
                "forklift_timeout_ms": 8000,
                "command_timeout_ms": 5000,
            }]
        )
    ])


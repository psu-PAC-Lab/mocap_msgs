from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    listener_node = Node(
        package="jetmax_optitrack_feedback",
        executable="jetmax_feedback"
    )

    ld.add_action(listener_node)

    return ld
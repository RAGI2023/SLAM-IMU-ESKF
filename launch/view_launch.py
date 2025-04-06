import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # 启动第一个节点 eskf_node
        Node(
            package='eskf',  # ESKF 功能包名称
            executable='eskf',  # ESKF 节点名称
            name='eskf',  # 节点名称
            output='screen',  # 输出到屏幕
            emulate_tty=True,  # 使输出易读
        ),
        
        # 启动第二个节点 another_node
        Node(
            package='eskf',  # 另一个功能包名称
            executable='errorViewer.py',  # 另一个节点的名称
            name='view',  # 节点名称
            output='screen',  # 输出到屏幕
            emulate_tty=True,  # 使输出易读
        ),
    ])
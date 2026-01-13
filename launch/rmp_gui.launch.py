import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    
    # ==========================================================
    # 1. 定义转发节点 (Relay)
    # 解决：SLAM 想要 /scan，但机器人只有 /lidar_1/scan_filtered
    # ==========================================================
    relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='scan_relay',
        output='screen',
        arguments=['/lidar_1/scan_filtered', '/scan'],
        parameters=[{'use_sim_time': True}]
    )

    # ==========================================================
    # 2. 定义 SLAM Toolbox
    # 既然机器人没启动它，我们就自己启动！
    # ==========================================================
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': 'True'}.items()
    )

    # ==========================================================
    # 3. 定义你的 GUI 节点
    # ==========================================================
    gui_node = Node(
        package='rmp_gui',
        executable='start_gui',
        output='screen',
        emulate_tty=True, # 允许直接打印 Log
        parameters=[
            {'use_sim_time': True} # 确保时间同步
        ],
        remappings=[
            ('/map', '/map'),
            # 如果你的 GUI 想要直接显示雷达点云，也可以加上这个：
            ('/scan', '/lidar_1/scan_filtered') 
        ]
    )

    return LaunchDescription([
        relay_node,          # 先搭桥
        slam_toolbox_launch, # 再启动算法
        gui_node             # 最后启动界面
    ])
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    
    # 获取配置文件路径
    # 假设你的配置文件放在 rmp_gui 包下的 config 目录里
    config_path = PathJoinSubstitution([
        FindPackageShare('rmp_gui'),
        'config',
        'robot_params.yaml'
    ])

    # 1. 定义转发节点 (Relay)
    relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='scan_relay',
        output='screen',
        arguments=['/lidar_1/scan_filtered', '/scan'],
        parameters=[{'use_sim_time': True}]
    )

    # 2. 定义 SLAM Toolbox
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

    # 3. 定义你的 GUI 节点 (已修改)
    gui_node = Node(
        package='rmp_gui',
        executable='start_gui',
        name='teleop_gui', # 必须与 YAML 文件中的节点名对应
        output='screen',
        emulate_tty=True,
        parameters=[
            config_path,       # 加载外部 YAML 参数文件
            {'use_sim_time': True} 
        ],
        remappings=[
            ('/map', '/map'),
            ('/scan', '/lidar_1/scan_filtered') 
        ]
    )

    return LaunchDescription([
        relay_node,
        slam_toolbox_launch,
        gui_node
    ])
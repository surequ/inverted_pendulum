from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Gazebo起動をlaunchファイルから呼び出し
    # gazebo_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         FindPackageShare('gazebo_ros'),
    #         '/launch/gazebo.launch.py'
    #     ]),
    #     launch_arguments={'verbose': 'true'}.items()
    # )
    # gazebo_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         FindPackageShare('ros_gz_sim'),
    #         '/gz_sim.launch.py'
    #     ])
    #     )
    world = os.path.join(
        get_package_share_directory('inverted_pendulum'),
        'worlds',
        'my_world.sdf'
    )
    
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['v4 ', world], 'on_exit_shutdown': 'false'}.items()
    )
    
    robot_description = Command([
        'cat ',
        PathJoinSubstitution([FindPackageShare('inverted_pendulum'), 'urdf', 'invertedpendulum.urdf'])
    ])

    #controller_path
    # controller = Command([
    #     'cat ',
    #     PathJoinSubstitution([FindPackageShare('inverted_pendulum'), 'controller', 'tire_controller.yaml'])
    # ])
    controller = os.path.join(
            get_package_share_directory('inverted_pendulum'),
            'controller',
            'tire_controller.yaml'
        )
    # robot_state_publisherノード
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # URDFファイルのパス(inverted_pendulum)
    inverted_pendulum_urdf_file = os.path.join(
        get_package_share_directory('inverted_pendulum'),  # ←自分のパッケージ名
        'urdf',
        'invertedpendulum.urdf'
    )

    # spawn_inverted_pendulum_entity_node = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=[
    #         '-entity', 'invertedpendulum',   # Gazebo 上での名前
    #         '-file', inverted_pendulum_urdf_file     # URDF ファイルパス
    #     ],
    #     output='screen'
    # )

    spawn_inverted_pendulum_entity_node = Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_invertedpendulum',
            output='screen',
            arguments=[
                '-file', inverted_pendulum_urdf_file,
                '-name', 'invertedpendulum',
                '-x', '0',
                '-y', '0',
                '-z', '0.5'
            ]
        )

    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            #arguments=['-d', rviz_config_file],  # 設定ファイルがあれば指定
            output='screen'
        )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_inverted_pendulum_entity_node,
        rviz2,
        control_node])
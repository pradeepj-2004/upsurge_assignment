from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch.substitutions import PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import os

# This will only take in effect if you are running in Simulation
# os.environ['GAZEBO_MODEL_PATH'] = os.path.join(get_package_share_directory('champ_gazebo'),
#                                                'models')


def generate_launch_description():


    champ_description_share_dir = get_package_share_directory(
        'go2_description')

    use_simulator = LaunchConfiguration('use_simulator')
    use_sim_time = LaunchConfiguration('use_sim_time')
    links_config = os.path.join(champ_description_share_dir, "config/links/links.yaml")


    declare_use_simulator = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='whether to use Gazebo Simulation.')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='whether to use or not sim time.')



    xacro_file_name = 'xacro/robot.xacro'

    xacro_full_dir = os.path.join(
        champ_description_share_dir, xacro_file_name)


    declare_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{"use_sim_time": use_sim_time},
                    {'robot_description': Command(['xacro ', xacro_full_dir])}],
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')])

    # SPAWN THE ROBOT TO GAZEBO IF use_simulator, FROM THE TOPIC "robot_description"
    declare_spawn_entity_to_gazebo_node = Node(package='gazebo_ros',
                                               condition=IfCondition(
                                                   use_simulator),
                                               executable='spawn_entity.py',
                                               parameters=[
                                                   {"use_sim_time": use_sim_time}],
                                               arguments=[
                                                   '-entity', '',
                                                   '-topic', '/robot_description', "-z", "0.2"],
                                               output='screen')
    world = LaunchConfiguration('world')
    # Launch argument for world file
    # START GAZEBO ONLY IF use_simulator IS SET TO TRUE
    declare_gazebo_world = DeclareLaunchArgument(
        "world", default_value=os.path.join(champ_description_share_dir, "worlds/default.world")
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            # 'pause': 'false'
            'world': world

        }.items(),
        condition=IfCondition(PythonExpression([use_simulator]))
    )
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_states_controller'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_group_effort_controller'],
        output='screen'
    )



    return LaunchDescription([
        declare_use_simulator,
        declare_use_sim_time,
        declare_robot_state_publisher_node,
        declare_gazebo_world,
        gazebo_server,
        gazebo_client,
        declare_spawn_entity_to_gazebo_node,
        load_joint_state_controller,
        load_joint_trajectory_controller,
    ])
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('hex_docking')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg, 'worlds', 'dock_world.sdf')
    model_file = os.path.join(pkg, 'models', 'hex_bot.sdf')

    resource_path = os.pathsep.join([
        os.path.join(pkg, 'models'),
        os.path.join(pkg, 'worlds'),
        os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ])

    # Just spawn positions -- this is NOT the docking formation.
    # Docking relationships (who docks to whom) live only in docking_node.py's FORMATION list.
    robots = [
    {'name': 'robot1', 'x': '0.0',  'y': '0.0'},
    {'name': 'robot2', 'x': '2.5',  'y': '0.0'},
    {'name': 'robot3', 'x': '-2.5', 'y': '0.0'},
    {'name': 'robot4', 'x': '0.0',  'y': '2.5'},
    {'name': 'robot5', 'x': '0.0',  'y': '-2.5'},
    {'name': 'robot6', 'x': '4.0',  'y': '0.0'},   
    {'name': 'robot7', 'x': '-4.0', 'y': '0.0'},
]

    actions = [
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
    ]

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    ))

    for i, r in enumerate(robots):
        ns = r['name']
        delay = 2.0 + i * 1.0

        actions.append(TimerAction(period=delay, actions=[Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-name', ns,
                '-file', model_file,
                '-x', r['x'],
                '-y', r['y'],
                '-z', '0.1',
            ],
        )]))

        actions.append(TimerAction(period=delay + 0.3, actions=[Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'{ns}_bridge',
            namespace=ns,
            output='screen',
            arguments=[
                f'/model/{ns}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                f'/model/{ns}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            ],
            remappings=[
                (f'/model/{ns}/cmd_vel', 'cmd_vel'),
                (f'/model/{ns}/odometry', 'odom'),
            ],
        )]))

    # World-frame ground-truth pose bridge (one, not per-robot)
    actions.append(TimerAction(period=6.0 + len(robots) * 1.0, actions=[Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='world_pose_bridge',
        output='screen',
        arguments=[
            '/world/dock_world/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
    )]))

    return LaunchDescription(actions)
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    package_name = 'slam_bot'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Start Ignition Gazebo with the empty.world file
    start_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', os.path.join(get_package_share_directory(package_name), 'worlds', 'empty.world')],
        output='screen'
    )

    # Launch the SLAM toolbox
    start_slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py'
        )]), launch_arguments={
            'slam_params_file': os.path.join(get_package_share_directory(package_name), 'config', 'mapper_params_online_async.yaml'),
            'use_sim_time': 'true'
        }.items()
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_ign_gazebo', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'slam_bot'],
                        output='screen')

    # Run the ros_gz_bridges
    gz_bridges = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'
        ],
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        start_gazebo,
        start_slam_toolbox,
        gz_bridges,
        spawn_entity,
    ])

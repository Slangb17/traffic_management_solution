from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace to push all topics into.')

    declare_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    launch_ira_laser_tools = Node(
        package='ira_laser_tools',
        name='mir_laser_scan_merger',
        executable='laserscan_multi_merger',
        parameters=[{'laserscan_topics': "b_scan f_scan",
                        'destination_frame': "virtual_laser_link",
                        'scan_destination_topic': "scan",
                        'cloud_destination_topic': "scan_cloud",
                        'min_height': -0.25,
                        'max_completion_time': 0.05,
                        'max_merge_time_diff': 0.005,
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'best_effort': False}],
        namespace=LaunchConfiguration('namespace'),    # adds namespace to topic names and frames
        output='screen')

    ld.add_action(declare_namespace_arg)
    ld.add_action(declare_sim_time_arg)

    ld.add_action(launch_ira_laser_tools)

    return ld

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, \
                           SetLaunchConfiguration, GroupAction, SetEnvironmentVariable, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart, OnExecutionComplete
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    traffic_management_dir = get_package_share_directory('traffic_management')
    mir_description_dir = get_package_share_directory('mir_description')

    namespace = LaunchConfiguration('namespace')
    prefix = LaunchConfiguration('prefix')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_yaw = LaunchConfiguration('robot_yaw')
    teleop_enabled = LaunchConfiguration('teleop_enabled')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    params_file = LaunchConfiguration('params_file')
    base_frame_id = LaunchConfiguration('base_frame_id')
    base_link_frame_id = LaunchConfiguration('base_link_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    scan_topic = LaunchConfiguration('scan_topic')
    autostart = LaunchConfiguration('autostart')
    robot_name = LaunchConfiguration('robot_name')
    map_topic = LaunchConfiguration('map_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    tf_topic = LaunchConfiguration('tf_topic')
    tf_static_topic = LaunchConfiguration('tf_static_topic')
    default_nav_to_pose_bt_xml = LaunchConfiguration('default_nav_to_pose_bt_xml')
    default_nav_through_pose_bt_xml = LaunchConfiguration('default_nav_through_pose_bt_xml')
    traffic_lanes_yaml = LaunchConfiguration('traffic_lanes_yaml')
    traffic_directions_file = LaunchConfiguration('traffic_directions_file')

    command_topic = LaunchConfiguration('command_topic')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    #remappings = [('/tf', 'tf'),
    #              ('/tf_static', 'tf_static')]
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
        'default_nav_through_poses_bt_xml': default_nav_through_pose_bt_xml,
        'x': LaunchConfiguration('robot_x'),
        'y': LaunchConfiguration('robot_y'),
        'yaw': LaunchConfiguration('robot_yaw'),
        'odom_frame_id': LaunchConfiguration('odom_frame_id'),
        'global_frame': 'map',
        'robot_base_frame': LaunchConfiguration('base_link_frame_id'),
        'odom_topic': LaunchConfiguration('odom_topic'),
        'map_topic': LaunchConfiguration('map_topic'),
        'scan_topic': LaunchConfiguration('scan_topic'), 
        'topic': LaunchConfiguration('scan_topic'),
        'traffic_lanes_yaml': LaunchConfiguration('traffic_lanes_yaml'),
        'traffic_directions_file': LaunchConfiguration('traffic_directions_file'),
        'robot_name': LaunchConfiguration('robot_name')
    }

    param_substitutions_controller = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
        'default_nav_through_poses_bt_xml': default_nav_through_pose_bt_xml,
        'x': LaunchConfiguration('robot_x'),
        'y': LaunchConfiguration('robot_y'),
        'yaw': LaunchConfiguration('robot_yaw'),
        'global_frame': LaunchConfiguration('odom_frame_id'),
        'robot_base_frame': LaunchConfiguration('base_link_frame_id'),
        'map_topic': LaunchConfiguration('map_topic'),
        'scan_topic': LaunchConfiguration('scan_topic'), 
        'topic': LaunchConfiguration('scan_topic'),
        'traffic_lanes_yaml': LaunchConfiguration('traffic_lanes_yaml'),
        'traffic_directions_file': LaunchConfiguration('traffic_directions_file'),
        'robot_name': LaunchConfiguration('robot_name')
        #'odom_topic': LaunchConfiguration('odom_topic')
    }

    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        root_key=LaunchConfiguration('namespace'),
        param_rewrites=param_substitutions,
        convert_types=True)
    
    configured_params_controller = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        root_key=LaunchConfiguration('namespace'),
        param_rewrites=param_substitutions_controller,
        convert_types=True)
    
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace to push all topics into.')
    
    declare_robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='mir_robot',
        description='Name of the robot')
    
    declare_prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='',
        description='Robot prefix.')

    declare_robot_x_arg = DeclareLaunchArgument(
        'robot_x',
        default_value='0.0',
        description='Spawning position of robot (x)')

    declare_robot_y_arg = DeclareLaunchArgument(
        'robot_y',
        default_value='0.0',
        description='Spawning position of robot (y)')

    declare_robot_yaw_arg = DeclareLaunchArgument(
        'robot_yaw',
        default_value='0.0',
        description='Spawning position of robot (yaw)')

    declare_teleop_arg = DeclareLaunchArgument(
        'teleop_enabled',
        default_value='true',
        description='Set to true to enable teleop to manually move MiR around.')
    
    declare_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(traffic_management_dir, 'config', 'mir_spawner_params_with_traffic_management.yaml'),
        # default_value=os.path.join(traffic_management_dir, 'config', 'mir_spawner_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_base_frame_id_arg = DeclareLaunchArgument(
        'base_frame_id',
        default_value='base_footprint',
        description='Base footprint frame')
    
    declare_base_link_frame_id_arg = DeclareLaunchArgument(
        'base_link_frame_id',
        default_value='base_link',
        description='Base_link frame')

    declare_odom_frame_id_arg = DeclareLaunchArgument(
        'odom_frame_id',
        default_value='odom',
        description='Odom frame')

    declare_scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='scan',
        description='Scan topic')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')
    
    declare_cmd_vel_cmd = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='cmd_vel',
        description='Define cmd_vel topic')

    declare_bt_nav_cmd = DeclareLaunchArgument(
        'default_nav_to_pose_bt_xml', default_value='')

    declare_bt_nav_through_cmd = DeclareLaunchArgument(
        'default_nav_through_pose_bt_xml', default_value='')
    
    declare_odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/odom'
    )

    declare_traffic_lanes_yaml_arg = DeclareLaunchArgument(
        'traffic_lanes_yaml', 
        default_value=os.path.join(traffic_management_dir, 'maps', 'tests_mini_warehouse_traffic_lanes.yaml')
    )

    declare_traffic_directions_file_arg = DeclareLaunchArgument(
        'traffic_directions_file', 
        default_value=os.path.join(traffic_management_dir, 'maps', 'tests_mini_warehouse_traffic_directions.png')
    )
       
            
    def process_namespace(context):
        robot_name = 'mir_robot'
        odom_frame_id = ''
        cmd_topic = ''
        base_link = ''
        odom_topic_name = ''
        map_topic = ''
        tf_topic = ''
        tf_static_topic = ''
        scan_topic = ''

        try:
            namespace = context.launch_configurations['namespace']
            robot_name = namespace + '/' + robot_name
            odom_frame_id =  namespace + '/' + context.launch_configurations['odom_frame_id']
            base_link = namespace + '/' + context.launch_configurations['base_link_frame_id']
            cmd_topic = namespace + '/' + context.launch_configurations['cmd_vel_topic']
            odom_topic_name = '/' + namespace + '/odom'
            map_topic = '/' + namespace + '/map'
            tf_topic = '/' + namespace + '/tf'
            tf_static_topic = '/' + namespace + '/tf_static'
            scan_topic = '/' + namespace + '/scan'
        except KeyError:
            print('WARNING!!!!! KEY ERROR')

        print('namespace', namespace)
        print('robot_name', robot_name)
        print('odom_frame_id', odom_frame_id)
        print('base_link', base_link)
        print('cmd_topic', cmd_topic)
        print('odom_topic_name', odom_topic_name)
        print('map_topic', map_topic)
        print('tf_topic', tf_topic)
        print('tf_static_topic', tf_static_topic)
        print('scan_topic', scan_topic)
        return [
            SetLaunchConfiguration('robot_name', robot_name),
            SetLaunchConfiguration('odom_frame_id', odom_frame_id), 
            SetLaunchConfiguration('base_link_frame_id', base_link),
            SetLaunchConfiguration('command_topic', cmd_topic),
            SetLaunchConfiguration('odom_topic', odom_topic_name),
            SetLaunchConfiguration('map_topic', map_topic),
            SetLaunchConfiguration('tf_topic', tf_topic),
            SetLaunchConfiguration('tf_static_topic', tf_static_topic),
            SetLaunchConfiguration('scan_topic', scan_topic)
        ]

    launch_mir_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_description_dir, 'launch', 'mir_launch.py')),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'prefix': LaunchConfiguration('namespace')
        }.items()
    )

    launch_mir_gazebo_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(traffic_management_dir, 'launch',
                         'include', 'mir_gazebo_common.py')),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace')
        }.items()
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', LaunchConfiguration('robot_name'),
                   '-topic', 'robot_description',
                   '-b', # bond node to gazebo model,
                   '-x', LaunchConfiguration('robot_x'),
                   '-y', LaunchConfiguration('robot_y'),
                   '-Y', LaunchConfiguration('robot_yaw')],  
        namespace=LaunchConfiguration('namespace'),
        output='screen')

    launch_teleop = Node(
       condition=IfCondition(LaunchConfiguration("teleop_enabled")),
       package='teleop_twist_keyboard',
       executable='teleop_twist_keyboard',
       namespace=LaunchConfiguration('namespace'),
       output='screen',
       prefix='xterm -e')
    
    static_transform = Node(
            package='tf2_ros',
            namespace = namespace,
            name="tf_static_map_to_odom",
            executable='static_transform_publisher',
            arguments= ["0", "0", "0", "0", "0", "0", "map", odom_frame_id]
        )
    
    lifecycle_nodes = [#'amcl',
                        'velocity_smoother',
                        'behavior_server',
                        'smoother_server',
                        'bt_navigator',
                        'waypoint_follower',
                        'planner_server',
                        'controller_server']
                        
    
    load_nodes = GroupAction(
        actions=[
            SetRemap(src=tf_topic, dst='/tf'),
            SetRemap(src=tf_static_topic, dst='/tf_static'),
            SetRemap(src=map_topic, dst='/map'),
            # Node(
            #     package='nav2_amcl',
            #     executable='amcl',
            #     name='amcl',
            #     output='screen',
            #     respawn=LaunchConfiguration('use_respawn'),
            #     respawn_delay=2.0,
            #     parameters=[configured_params],
            #     arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            #     remappings=remappings,
            #     namespace=LaunchConfiguration('namespace')),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params_controller],
                arguments=['--ros-args', '--log-level', log_level],
                #remappings=remappings,
                namespace=LaunchConfiguration('namespace')),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                #remappings=remappings,
                namespace=LaunchConfiguration('namespace')),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                #remappings=remappings,
                namespace=LaunchConfiguration('namespace')),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                #remappings=remappings,
                namespace=LaunchConfiguration('namespace')),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                #remappings=remappings,
                namespace=LaunchConfiguration('namespace')),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                #remappings=remappings,
                namespace=LaunchConfiguration('namespace')),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                #remappings=remappings, #+
                        #[('cmd_vel', command_topic), ('cmd_vel_smoothed', 'cmd_vel')],
                namespace=LaunchConfiguration('namespace')),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                            {'autostart': LaunchConfiguration('autostart')},
                            {'node_names': lifecycle_nodes}],
                #remappings=remappings,
                namespace=LaunchConfiguration('namespace'))
        ]
    )
    
    #start_seq = TimerAction(period=10.0, actions=[load_nodes])

    ld = LaunchDescription()
    
    ld.add_action(declare_namespace_arg)
    ld.add_action(declare_robot_name_arg)
    ld.add_action(declare_prefix_arg)
    ld.add_action(declare_robot_x_arg)
    ld.add_action(declare_robot_y_arg)
    ld.add_action(declare_robot_yaw_arg)
    ld.add_action(declare_teleop_arg)
    ld.add_action(declare_sim_time_arg)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_base_frame_id_arg)
    ld.add_action(declare_base_link_frame_id_arg)
    ld.add_action(declare_odom_frame_id_arg)
    ld.add_action(declare_scan_topic_arg)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bt_nav_cmd)
    ld.add_action(declare_bt_nav_through_cmd)
    ld.add_action(declare_cmd_vel_cmd)
    ld.add_action(declare_traffic_lanes_yaml_arg)
    ld.add_action(declare_traffic_directions_file_arg)

    ld.add_action(OpaqueFunction(function=process_namespace))

    ld.add_action(launch_mir_description)
    ld.add_action(launch_mir_gazebo_common)
    ld.add_action(spawn_robot)
    ld.add_action(static_transform)
    #ld.add_action(launch_teleop)

    #ld.add_action(start_seq)
    
    ld.add_action(load_nodes)

    return ld

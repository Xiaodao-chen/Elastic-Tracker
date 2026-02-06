import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer
import launch_ros.actions
import launch_ros.descriptions
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    map_size_x = LaunchConfiguration('map_size_x', default='42.0')
    map_size_y = LaunchConfiguration('map_size_y', default='40.0')
    map_size_z = LaunchConfiguration('map_size_z', default='5.0')
    use_mockamap = LaunchConfiguration('use_mockamap', default='true')
    use_dynamic = LaunchConfiguration('use_dynamic', default='true')
    
    map_size_x_arg = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_arg = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_arg = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    use_mockamap_arg = DeclareLaunchArgument('use_mockamap', default_value=use_mockamap, description='Use mockamap for map generation')
    use_dynamic_arg = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic, description='Use dynamic simulation')
    
    # Get config file paths
    tracker_config = os.path.join(
        get_package_share_directory('tracker_fsm'),
        'config',
        'tracker_sim.yaml'
    )
    
    nav_config = os.path.join(
        get_package_share_directory('tracker_fsm'),
        'config',
        'nav_sim.yaml'
    )
    
    camera_file = os.path.join(
        get_package_share_directory('local_sensing'),
        'config',
        'camera.yaml'
    )
    
    gains_file = os.path.join(
        get_package_share_directory('so3_control'),
        'config',
        'gains_hummingbird.yaml'
    )
    
    corrections_file = os.path.join(
        get_package_share_directory('so3_control'),
        'config',
        'corrections_hummingbird.yaml'
    )
    
    # Mockamap node (shared map)
    mockamap_node = Node(
        package='mockamap',
        executable='mockamap_node',
        name='mockamap_node',
        output='screen',
        remappings=[
            ('/mock_map', '/map_generator/global_cloud')
        ],
        parameters=[
            {'seed': 510},
            {'update_freq': 1.0},
            {'resolution': 0.1},
            {'x_length': PythonExpression(['int(', map_size_x, ')'])},
            {'y_length': PythonExpression(['int(', map_size_y, ')'])},
            {'z_length': PythonExpression(['int(', map_size_z, ')'])},
            {'type': 2},
            {'width_min': 0.5},
            {'width_max': 1.5},
            {'height_min': 3.5},
            {'height_max': 4.5},
            {'obstacle_number': 120}
        ],
        condition=IfCondition(use_mockamap)
    )
    
    # ========== TRACKER DRONE GROUP (drone0 namespace) ==========
    tracker_drone_group = GroupAction([
        # SO3 Quadrotor Simulator
        Node(
            package='so3_quadrotor_simulator',
            executable='so3_quadrotor_simulator',
            name='so3_quadrotor_simulator',
            namespace='drone0',
            output='screen',
            parameters=[
                {'rate/odom': 100.0},
                {'simulator/init_state_x': -15.0},
                {'simulator/init_state_y': 0.0},
                {'simulator/init_state_z': 0.1}
            ],
            remappings=[
                ('odom', '/drone0/odom'),
                ('cmd', '/drone0/so3_cmd')
            ],
            condition=IfCondition(use_dynamic)
        ),
        
        # SO3 Control
        ComposableNodeContainer(
            name='so3_control_container',
            namespace='drone0',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                launch_ros.descriptions.ComposableNode(
                    package='so3_control',
                    plugin='SO3ControlComponent',
                    name='so3_control_component',
                    parameters=[
                        {'so3_control/init_state_x': -15.0},
                        {'so3_control/init_state_y': 0.0},
                        {'so3_control/init_state_z': 0.1},
                        {'mass': 0.98},
                        {'use_angle_corrections': False},
                        {'use_external_yaw': False},
                        {'gains/rot/z': 1.0},
                        {'gains/ang/z': 0.1},
                        gains_file,
                        corrections_file
                    ],
                    remappings=[
                        ('odom', '/drone0/odom'),
                        ('position_cmd', '/drone0/position_cmd'),
                        ('so3_cmd', '/drone0/so3_cmd')
                    ]
                )
            ],
            output='screen',
            condition=IfCondition(use_dynamic)
        ),
        
        # Local Sensing (PCL Render Node)
        Node(
            package='local_sensing',
            executable='pcl_render_node',
            name='pcl_render_node',
            namespace='drone0',
            output='screen',
            parameters=[
                {'sensing_horizon': 5.0},
                {'sensing_rate': 30.0},
                {'estimation_rate': 30.0},
                {'map/x_size': map_size_x},
                {'map/y_size': map_size_y},
                {'map/z_size': map_size_z},
                camera_file
            ],
            remappings=[
                ('global_map', '/map_generator/global_cloud'),
                ('odometry', '/drone0/odom'),
                ('pcl_render_node/cloud', '/drone0/pcl_render_node/cloud'),
                ('depth', '/drone0/pcl_render_node/depth'),
                ('camera_pose', '/drone0/pcl_render_node/camera_pose')
            ]
        ),
        
        # Odometry Visualization (drone0 as car DAE, matching ROS1 simulation1.launch)
        Node(
            package='odom_visualization',
            executable='odom_visualization',
            name='odom_visualization',
            namespace='drone0',
            output='screen',
            parameters=[
                {'color/a': 1.0},
                {'color/r': 0.0},
                {'color/g': 0.0},
                {'color/b': 1.0},
                {'robot_scale': 0.05},
                {'mesh_resource': 'package://odom_visualization/meshes/car.dae'}
            ],
            remappings=[
                ('odom', '/drone0/odom')
            ]
        ),
        
        # Tracker Node (FSM)
        Node(
            package='tracker_fsm',
            executable='tracker_node',
            name='tracker_node',
            namespace='drone0',
            output='screen',
            parameters=[tracker_config],
            remappings=[
                # FSM subscriptions
                ('odom', '/drone0/odom'),
                ('target', '/target/target_odom'),
                ('gridmap_inflate', '/drone0/gridmap_inflate'),
                ('triger', '/triger'),
                # FSM publications
                ('trajectory', '/drone0/trajectory'),
                ('position_cmd', '/drone0/position_cmd'),
                # Target EKF publications (created within tracker_node)
                ('target_odom', '/target/target_odom'),
                # Target EKF subscriptions
                ('yolo', '/target/odom'),  # In simulation, directly use target's true odom
                # Mapper subscriptions (created within tracker_node)
                ('depth', '/drone0/pcl_render_node/depth'),
                ('global_map', '/map_generator/global_cloud')
            ]
        ),
        
        # Trajectory Server
        Node(
            package='tracker_fsm',
            executable='traj_server',
            name='traj_server',
            namespace='drone0',
            output='screen',
            parameters=[
                {'traj_server/time_forward': 1.0}
            ],
            remappings=[
                ('trajectory', '/drone0/trajectory'),
                ('position_cmd', '/drone0/position_cmd'),
                ('heartbeat', '/drone0/heartbeat')
            ]
        )
    ])
    
    # ========== TARGET DRONE GROUP (target namespace) ==========
    target_drone_group = GroupAction([
        # SO3 Quadrotor Simulator (target)
        Node(
            package='so3_quadrotor_simulator',
            executable='so3_quadrotor_simulator',
            name='so3_quadrotor_simulator',
            namespace='target',
            output='screen',
            parameters=[
                {'rate/odom': 100.0},
                {'simulator/init_state_x': 2.0},
                {'simulator/init_state_y': 0.0},
                {'simulator/init_state_z': 1.0}
            ],
            remappings=[
                ('odom', '/target/odom'),
                ('cmd', '/target/so3_cmd')
            ],
            condition=IfCondition(use_dynamic)
        ),
        
        # SO3 Control (target)
        ComposableNodeContainer(
            name='so3_control_container',
            namespace='target',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                launch_ros.descriptions.ComposableNode(
                    package='so3_control',
                    plugin='SO3ControlComponent',
                    name='so3_control_component',
                    parameters=[
                        {'so3_control/init_state_x': 2.0},
                        {'so3_control/init_state_y': 0.0},
                        {'so3_control/init_state_z': 1.0},
                        {'mass': 0.98},
                        {'use_angle_corrections': False},
                        {'use_external_yaw': False},
                        {'gains/rot/z': 1.0},
                        {'gains/ang/z': 0.1},
                        gains_file,
                        corrections_file
                    ],
                    remappings=[
                        ('odom', '/target/odom'),
                        ('position_cmd', '/target/position_cmd'),
                        ('so3_cmd', '/target/so3_cmd')
                    ]
                )
            ],
            output='screen',
            condition=IfCondition(use_dynamic)
        ),
        
        # Local Sensing (PCL Render Node) for target
        Node(
            package='local_sensing',
            executable='pcl_render_node',
            name='pcl_render_node',
            namespace='target',
            output='screen',
            parameters=[
                {'sensing_horizon': 5.0},
                {'sensing_rate': 30.0},
                {'estimation_rate': 30.0},
                {'map/x_size': map_size_x},
                {'map/y_size': map_size_y},
                {'map/z_size': map_size_z},
                camera_file
            ],
            remappings=[
                ('global_map', '/map_generator/global_cloud'),
                ('odometry', '/target/odom'),
                ('pcl_render_node/cloud', '/target/pcl_render_node/cloud'),
                ('depth', '/target/pcl_render_node/depth'),
                ('camera_pose', '/target/pcl_render_node/camera_pose')
            ]
        ),
        
        # Odometry Visualization (target as car DAE, matching ROS1 fake_car_target.launch)
        Node(
            package='odom_visualization',
            executable='odom_visualization_car',
            name='odom_visualization_car',
            namespace='target',
            output='screen',
            parameters=[
                {'color/a': 1.0},
                {'color/r': 1.0},
                {'color/g': 0.0},
                {'color/b': 0.0},
                {'robot_scale': 0.1},
                {'mesh_resource': 'package://odom_visualization/meshes/car.dae'}
            ],
            remappings=[
                ('odom', '/target/odom')
            ]
        ),
        
        # Tracker Node (Navigation mode with fake=true)
        Node(
            package='tracker_fsm',
            executable='tracker_node',
            name='tracker_node',
            namespace='target',
            output='screen',
            parameters=[nav_config],
            remappings=[
                # FSM subscriptions
                ('odom', '/target/odom'),
                ('gridmap_inflate', '/target/gridmap_inflate'),
                ('triger', '/move_base_simple/goal'),
                # FSM publications
                ('trajectory', '/target/trajectory'),
                ('position_cmd', '/target/position_cmd'),
                # Mapper subscriptions (created within tracker_node)
                ('depth', '/target/pcl_render_node/depth'),
                ('global_map', '/map_generator/global_cloud')
            ]
        ),
        
        # Trajectory Server (target)
        Node(
            package='tracker_fsm',
            executable='traj_server',
            name='traj_server',
            namespace='target',
            output='screen',
            parameters=[
                {'traj_server/time_forward': 1.0}
            ],
            remappings=[
                ('trajectory', '/target/trajectory'),
                ('position_cmd', '/target/position_cmd'),
                ('heartbeat', '/target/heartbeat')
            ]
        )
    ])
    
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(map_size_x_arg)
    ld.add_action(map_size_y_arg)
    ld.add_action(map_size_z_arg)
    ld.add_action(use_mockamap_arg)
    ld.add_action(use_dynamic_arg)
    
    # Add nodes
    ld.add_action(mockamap_node)
    ld.add_action(tracker_drone_group)
    ld.add_action(target_drone_group)
    
    return ld

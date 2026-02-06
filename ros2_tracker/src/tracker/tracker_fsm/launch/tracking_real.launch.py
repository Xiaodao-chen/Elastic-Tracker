"""
Real-world deployment launch file for Elastic Tracker (ROS2).

Architecture:
  - Odometry: from VIO (e.g., VINS-Fusion, T265, or EKF)
  - Depth: from RealSense D435 (16UC1 or 32FC1)
  - Target: position in world frame (converted from local frame externally)
  - Mapping: local sensing via depth image + odometry (NOT global map)
  - Output: position_cmd (consumed by so3_control or converted to cmd_vel)

Usage:
  ros2 launch tracker_fsm tracking_real.launch.py

  # With custom topics:
  ros2 launch tracker_fsm tracking_real.launch.py \\
      odom_topic:=/vins_fusion/odom \\
      depth_topic:=/camera/depth/image_rect_raw \\
      target_topic:=/target_odom \\
      triger_topic:=/triger

  # To trigger tracking, publish a PoseStamped to /triger:
  ros2 topic pub --once /triger geometry_msgs/msg/PoseStamped \\
      '{header: {frame_id: "world"}, pose: {position: {x: 0, y: 0, z: 0.5}}}'
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ========== Launch arguments ==========
    odom_topic = LaunchConfiguration('odom_topic', default='/vio/odom')
    depth_topic = LaunchConfiguration('depth_topic', default='/camera/depth/image_rect_raw')
    target_topic = LaunchConfiguration('target_topic', default='/target_world_odom')
    triger_topic = LaunchConfiguration('triger_topic', default='/triger')

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/vio/odom',
        description='Odometry topic from VIO (nav_msgs/msg/Odometry)')
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic', default_value='/camera/depth/image_rect_raw',
        description='Depth image topic from RealSense (sensor_msgs/msg/Image, 16UC1 or 32FC1)')
    target_topic_arg = DeclareLaunchArgument(
        'target_topic', default_value='/target_world_odom',
        description='Target odometry in world frame (nav_msgs/msg/Odometry)')
    triger_topic_arg = DeclareLaunchArgument(
        'triger_topic', default_value='/triger',
        description='Trigger topic to start tracking (geometry_msgs/msg/PoseStamped)')

    # ========== Config file ==========
    tracker_config = os.path.join(
        get_package_share_directory('tracker_fsm'),
        'config',
        'tracker_real.yaml'
    )

    # ========== RViz ==========
    rviz_config = os.path.join(
        get_package_share_directory('tracker_fsm'),
        'launch',
        'default.rviz'
    )

    # ========== Nodes ==========
    return LaunchDescription([
        odom_topic_arg,
        depth_topic_arg,
        target_topic_arg,
        triger_topic_arg,

        # ---- RViz2 ----
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),

        # ---- Odometry Visualization (tracker drone with FOV) ----
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
                {'robot_scale': 0.08},
                {'mesh_resource': 'package://odom_visualization/meshes/car.dae'},
                {'mesh_yaw_offset_deg': 180.0}
            ],
            remappings=[
                ('odom', odom_topic)
            ]
        ),

        # ---- Tracker Node (FSM + Mapping + EKF + Planning) ----
        Node(
            package='tracker_fsm',
            executable='tracker_node',
            name='tracker_node',
            namespace='drone0',
            output='screen',
            parameters=[tracker_config],
            remappings=[
                # FSM subscriptions
                ('odom', odom_topic),
                ('target', '/drone0/target_odom'),
                ('gridmap_inflate', '/drone0/gridmap_inflate'),
                ('triger', triger_topic),
                # FSM publications
                ('trajectory', '/drone0/trajectory'),
                ('position_cmd', '/drone0/position_cmd'),
                # Target EKF subscriptions
                # 'yolo' is the detection input: in real world, use target odom in world frame
                ('yolo', target_topic),
                # Target EKF publications
                ('target_odom', '/drone0/target_odom'),
                # Mapper subscriptions (depth mode: depth image + odom are synced)
                ('depth', depth_topic),
                # Mapper publication
                ('gridmap_inflate', '/drone0/gridmap_inflate'),
                # Target mask
                ('target', '/drone0/target_odom'),
            ]
        ),

        # ---- Trajectory Server ----
        Node(
            package='tracker_fsm',
            executable='traj_server',
            name='traj_server',
            namespace='drone0',
            output='screen',
            parameters=[
                {'time_forward': 1.0}
            ],
            remappings=[
                ('position_cmd', '/drone0/position_cmd'),
                ('trajectory', '/drone0/trajectory'),
                ('heartbeat', '/drone0/heartbeat')
            ]
        ),
    ])

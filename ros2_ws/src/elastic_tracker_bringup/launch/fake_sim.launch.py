from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    bringup_share = FindPackageShare("elastic_tracker_bringup")

    mockamap_params = PathJoinSubstitution([bringup_share, "config", "mockamap.yaml"])
    so3_quadrotor_params = PathJoinSubstitution([bringup_share, "config", "so3_quadrotor.yaml"])
    so3_controller_params = PathJoinSubstitution([bringup_share, "config", "so3_controller.yaml"])
    mapping_params = PathJoinSubstitution([bringup_share, "config", "mapping_fake.yaml"])
    planning_params = LaunchConfiguration("planning_params")

    enable_local_sensing = LaunchConfiguration("enable_local_sensing")
    enable_rviz = LaunchConfiguration("enable_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_xvfb = LaunchConfiguration("use_xvfb")

    # Global map publisher
    mockamap = Node(
        package="mockamap",
        executable="mockamap_node",
        name="mockamap",
        output="screen",
        parameters=[mockamap_params],
        remappings=[
            ("mock_map", "/global_map"),
        ],
    )

    # Container with components, in ROS1 this was nodelet manager.
    container = ComposableNodeContainer(
        name="elastic_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="so3_quadrotor",
                plugin="so3_quadrotor::So3QuadrotorComponent",
                name="so3_quadrotor",
                namespace="drone0",
                parameters=[so3_quadrotor_params, {"init_x": 2.0, "init_y": 0.0, "init_z": 1.0}],
            ),
            ComposableNode(
                package="so3_controller",
                plugin="so3_controller::SO3ControllerComponent",
                name="so3_controller",
                namespace="drone0",
                parameters=[so3_controller_params],
                remappings=[
                    ("position_cmd", "position_cmd"),
                    ("odom", "odom"),
                    ("imu", "imu"),
                    ("so3cmd", "so3cmd"),
                ],
            ),
            ComposableNode(
                package="mapping",
                plugin="mapping::MappingComponent",
                name="mapping",
                namespace="drone0",
                parameters=[mapping_params],
                remappings=[
                    # mockamap publishes global topic
                    ("global_map", "/global_map"),
                ],
            ),
            ComposableNode(
                package="planning",
                plugin="planning::PlanningComponent",
                name="planning",
                namespace="drone0",
                parameters=[planning_params],
                # Keep trigger topics inside the drone namespace (/drone0/...), which is consistent
                # with the rest of the bringup and avoids relying on global topics.
            ),
        ],
    )

    traj_server = Node(
        package="planning",
        executable="traj_server_node",
        name="traj_server",
        namespace="drone0",
        output="screen",
        remappings=[
            ("trajectory", "trajectory"),
            ("heartbeat", "heartbeat"),
            ("position_cmd", "position_cmd"),
        ],
    )

    local_sensing = Node(
        package="local_sensing_node",
        executable="pcl_render_node",
        name="pcl_render_node",
        namespace="drone0",
        output="screen",
        parameters=[{
            "sensing_horizon": 5.0,
            "sensing_rate": 30.0,
            "estimation_rate": 30.0,
        }],
        remappings=[
            ("global_map", "/global_map"),
            ("odometry", "odom"),
            ("depth", "depth"),
        ],
        condition=IfCondition(enable_local_sensing),
    )

    mapping_vis = Node(
        package="mapping",
        executable="mapping_vis_node",
        name="mapping_vis",
        namespace="drone0",
        output="screen",
        parameters=[{"remove_floor_ceil": False}],
    )

    odom_vis = Node(
        package="odom_visualization",
        executable="odom_visualization_node",
        name="odom_visualization",
        namespace="drone0",
        output="screen",
        parameters=[
            {"robot_scale": 0.7},
            {"mesh_resource": "package://odom_visualization/meshes/f250.dae"},
        ],
        remappings=[
            ("odom", "odom"),
        ],
    )

    # RViz2 (optional).
    # - In headless/server environments, xvfb is the most reliable way to run rviz2.
    # - On a desktop with a real display, set use_xvfb:=false.
    rviz2_xvfb = ExecuteProcess(
        cmd=["xvfb-run", "-a", "rviz2", "-d", rviz_config],
        output="screen",
        condition=IfCondition(use_xvfb),
    )
    rviz2_gui = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=UnlessCondition(use_xvfb),
    )

    rviz_group = GroupAction(
        actions=[rviz2_xvfb, rviz2_gui],
        condition=IfCondition(enable_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("enable_local_sensing", default_value="false"),
        DeclareLaunchArgument("enable_rviz", default_value="false"),
        DeclareLaunchArgument("use_xvfb", default_value="true"),
        DeclareLaunchArgument(
            "planning_params",
            default_value=PathJoinSubstitution([bringup_share, "config", "planning_fake.yaml"]),
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([bringup_share, "config", "rviz_sim_ros2.rviz"]),
        ),
        mockamap,
        container,
        traj_server,
        local_sensing,
        mapping_vis,
        odom_vis,
        rviz_group,
    ])


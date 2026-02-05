from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    ns = LaunchConfiguration("ns")
    init_x = LaunchConfiguration("init_x")
    init_y = LaunchConfiguration("init_y")
    init_z = LaunchConfiguration("init_z")

    uav_share = FindPackageShare("uav_simulator")
    quad_params = PathJoinSubstitution([uav_share, "config", "so3_quadrotor.yaml"])
    ctrl_params = PathJoinSubstitution([uav_share, "config", "so3_controller.yaml"])
    mockamap_params = PathJoinSubstitution([uav_share, "config", "mockamap.yaml"])

    # ROS1 loaded camera intrinsics from local_sensing_node/params/camera.yaml
    ls_share = FindPackageShare("local_sensing_node")
    cam_params = PathJoinSubstitution([ls_share, "params", "camera.yaml"])

    container = ComposableNodeContainer(
        name="uav_container",
        namespace=ns,
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="so3_quadrotor",
                plugin="so3_quadrotor::So3QuadrotorComponent",
                name="so3_quadrotor",
                parameters=[quad_params, {"init_x": init_x, "init_y": init_y, "init_z": init_z}],
                remappings=[
                    ("odom", "odom"),
                    ("imu", "imu"),
                    ("so3cmd", "so3cmd"),
                    ("vis", "vis"),
                ],
            ),
            ComposableNode(
                package="so3_controller",
                plugin="so3_controller::SO3ControllerComponent",
                name="so3_controller",
                parameters=[ctrl_params],
                remappings=[
                    ("odom", "odom"),
                    ("imu", "imu"),
                    ("so3cmd", "so3cmd"),
                    ("position_cmd", "position_cmd"),
                ],
            ),
        ],
    )

    # local sensing (non-CUDA, ROS2 port) - topic names align with ROS1 uav_simulator.launch
    local_sensing = Node(
        package="local_sensing_node",
        executable="pcl_render_node",
        name="pcl_render_node",
        namespace=ns,
        output="screen",
        parameters=[
            cam_params,
            {
                "sensing_horizon": 5.0,
                "sensing_rate": 30.0,
                "estimation_rate": 30.0,
            },
        ],
        remappings=[
            ("global_map", "/global_map"),
            ("odometry", "odom"),
            ("depth", "depth"),
        ],
    )

    # global map source (ROS1: mockamap -> /global_map)
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

    return LaunchDescription([
        DeclareLaunchArgument("ns", default_value=""),
        DeclareLaunchArgument("init_x", default_value="0.0"),
        DeclareLaunchArgument("init_y", default_value="0.0"),
        DeclareLaunchArgument("init_z", default_value="0.2"),
        mockamap,
        container,
        local_sensing,
    ])


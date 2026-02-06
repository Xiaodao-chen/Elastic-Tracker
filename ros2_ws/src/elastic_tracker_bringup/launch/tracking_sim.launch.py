from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("elastic_tracker_bringup")
    fake_sim_launch = PathJoinSubstitution([bringup_share, "launch", "fake_sim.launch.py"])
    target_ekf_share = FindPackageShare("target_ekf")

    enable_rviz = LaunchConfiguration("enable_rviz")
    enable_local_sensing = LaunchConfiguration("enable_local_sensing")
    planning_params = LaunchConfiguration("planning_params")
    rviz_config = LaunchConfiguration("rviz_config")
    use_xvfb = LaunchConfiguration("use_xvfb")

    # ROS1-like initial states:
    # - tracker (drone0): init (0,0,0.2) by default (uav_simulator.launch defaults)
    # - target: init (2,0,1) by default (fake_target.launch)
    tracker_init_x = LaunchConfiguration("tracker_init_x")
    tracker_init_y = LaunchConfiguration("tracker_init_y")
    tracker_init_z = LaunchConfiguration("tracker_init_z")

    enable_fake_target = LaunchConfiguration("enable_fake_target")
    fake_target_mode = LaunchConfiguration("fake_target_mode")
    fake_target_rate_hz = LaunchConfiguration("fake_target_rate_hz")
    fake_target_radius = LaunchConfiguration("fake_target_radius")
    fake_target_omega = LaunchConfiguration("fake_target_omega")
    fake_target_center_x = LaunchConfiguration("fake_target_center_x")
    fake_target_center_y = LaunchConfiguration("fake_target_center_y")
    fake_target_center_z = LaunchConfiguration("fake_target_center_z")
    fake_target_static_x = LaunchConfiguration("fake_target_static_x")
    fake_target_static_y = LaunchConfiguration("fake_target_static_y")
    fake_target_static_z = LaunchConfiguration("fake_target_static_z")

    enable_target_ekf_sim = LaunchConfiguration("enable_target_ekf_sim")
    pitch_thr = LaunchConfiguration("pitch_thr")

    auto_trigger = LaunchConfiguration("auto_trigger")
    trigger_delay = LaunchConfiguration("trigger_delay")
    trigger_x = LaunchConfiguration("trigger_x")
    trigger_y = LaunchConfiguration("trigger_y")
    trigger_z = LaunchConfiguration("trigger_z")

    fake_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fake_sim_launch),
        launch_arguments={
            "enable_rviz": enable_rviz,
            "enable_local_sensing": enable_local_sensing,
            "planning_params": planning_params,
            "rviz_config": rviz_config,
            "use_xvfb": use_xvfb,
            "init_x": tracker_init_x,
            "init_y": tracker_init_y,
            "init_z": tracker_init_z,
        }.items(),
    )

    # Publishes /target/odom (nav_msgs/Odometry) as the "true target" in sim, like ROS1.
    fake_target = Node(
        package="target_ekf",
        executable="fake_target_pub_node",
        name="fake_target_pub",
        namespace="target",
        output="screen",
        parameters=[{
            "topic": "odom",
            "frame_id": "world",
            "mode": fake_target_mode,
            "rate_hz": fake_target_rate_hz,
            "radius": fake_target_radius,
            "omega": fake_target_omega,
            "center_x": fake_target_center_x,
            "center_y": fake_target_center_y,
            "center_z": fake_target_center_z,
            "static_x": fake_target_static_x,
            "static_y": fake_target_static_y,
            "static_z": fake_target_static_z,
        }],
        condition=IfCondition(enable_fake_target),
    )

    # ROS1-like: run target_ekf_sim to convert /target/odom + /drone0/odom -> /drone0/target
    target_ekf_sim = Node(
        package="target_ekf",
        executable="target_ekf_sim_node",
        name="target_ekf_sim",
        namespace="drone0",
        output="screen",
        parameters=[
            # Use a ROS2-style parameter file (ros__parameters), otherwise rcl will reject it.
            PathJoinSubstitution([target_ekf_share, "config", "camera_ros2.yaml"]),
            {"pitch_thr": pitch_thr},
        ],
        remappings=[
            ("yolo", "/target/odom"),
            ("odom", "odom"),
            # ROS1-compatible global name
            ("target_odom", "/target_ekf_odom"),
        ],
        condition=IfCondition(enable_target_ekf_sim),
    )

    # Optional: auto-send a one-shot trigger after a short delay.
    trigger_msg = PythonExpression([
        "'{header: {frame_id: world}, pose: {position: {x: ' + str(",
        trigger_x,
        ") + ', y: ' + str(",
        trigger_y,
        ") + ', z: ' + str(",
        trigger_z,
        ") + '}, orientation: {w: 1.0}}}'"
    ])
    trigger_cmd = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub", "--once",
            "/triger", "geometry_msgs/msg/PoseStamped",
            trigger_msg,
        ],
        output="screen",
    )
    trigger_timer = TimerAction(
        period=trigger_delay,
        actions=[trigger_cmd],
        condition=IfCondition(auto_trigger),
    )

    return LaunchDescription([
        # passthrough args to fake_sim.launch.py
        DeclareLaunchArgument("enable_local_sensing", default_value="false"),
        DeclareLaunchArgument("enable_rviz", default_value="true"),
        DeclareLaunchArgument("use_xvfb", default_value="auto"),
        DeclareLaunchArgument("tracker_init_x", default_value="2.0"),
        DeclareLaunchArgument("tracker_init_y", default_value="0.0"),
        DeclareLaunchArgument("tracker_init_z", default_value="1.0"),
        DeclareLaunchArgument(
            "planning_params",
            default_value=PathJoinSubstitution([bringup_share, "config", "planning_fake.yaml"]),
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([bringup_share, "config", "rviz_sim_ros2.rviz"]),
        ),

        # target (sim)
        DeclareLaunchArgument("enable_fake_target", default_value="true"),
        DeclareLaunchArgument("fake_target_mode", default_value="static"),  # circle|static
        DeclareLaunchArgument("fake_target_rate_hz", default_value="30.0"),
        DeclareLaunchArgument("fake_target_radius", default_value="5.0"),
        DeclareLaunchArgument("fake_target_omega", default_value="0.2"),
        # align with ROS1 fake_target.launch init: (2,0,1)
        DeclareLaunchArgument("fake_target_center_x", default_value="0.0"),
        DeclareLaunchArgument("fake_target_center_y", default_value="0.0"),
        DeclareLaunchArgument("fake_target_center_z", default_value="0.5"),
        DeclareLaunchArgument("fake_target_static_x", default_value="2.0"),
        DeclareLaunchArgument("fake_target_static_y", default_value="0.0"),
        DeclareLaunchArgument("fake_target_static_z", default_value="1.0"),

        # target ekf sim (ROS1-like)
        DeclareLaunchArgument("enable_target_ekf_sim", default_value="true"),
        DeclareLaunchArgument("pitch_thr", default_value="37.0"),

        # optional auto trigger
        DeclareLaunchArgument("auto_trigger", default_value="false"),
        DeclareLaunchArgument("trigger_delay", default_value="2.0"),
        DeclareLaunchArgument("trigger_x", default_value="0.0"),
        DeclareLaunchArgument("trigger_y", default_value="0.0"),
        DeclareLaunchArgument("trigger_z", default_value="1.0"),

        fake_sim,
        fake_target,
        target_ekf_sim,
        trigger_timer,
    ])


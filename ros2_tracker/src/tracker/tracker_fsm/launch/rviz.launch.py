import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config_path = os.path.join(
        get_package_share_directory('tracker_fsm'),
        'launch',
        'default.rviz'
    )

    # 准备环境变量
    env_vars = []
    
    # 默认启用软件渲染（解决 GLX 上下文问题）
    # 如果环境变量已设置，则使用环境变量的值；否则默认启用软件渲染
    # 如果系统有硬件 OpenGL 支持，可以在启动前设置：export LIBGL_ALWAYS_SOFTWARE=0
    if os.environ.get('LIBGL_ALWAYS_SOFTWARE', '1') != '0':
        env_vars.append(SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'))
        env_vars.append(SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe'))
        # 禁用 VSync 以提高性能
        env_vars.append(SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3'))
    
    # 禁用 FastRTPS 共享内存传输（解决 RTPS_TRANSPORT_SHM 警告）
    # 这些警告通常不影响功能，但可以通过环境变量禁用
    # 注意：这可能会影响性能，如果不需要可以注释掉
    # env_vars.append(SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', ''))

    rviz_node = launch_ros.actions.Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['--display-config', rviz_config_path])

    ld = LaunchDescription()
    # 先添加环境变量设置
    for env_var in env_vars:
        ld.add_action(env_var)
    ld.add_action(rviz_node)

    return ld

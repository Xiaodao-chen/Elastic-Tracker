# ROS2 Workspace (`ros2_ws`)

This workspace contains ROS2 (Jazzy) ports / decoupled-core versions of selected packages from the original ROS1 `src/` tree.

## Build

Use **system Python** (avoid conda).

For **ROS 2 Humble (Ubuntu 22.04)**, see: `RUN_HUMBLE.md`

Legacy note: the original instructions below were for sourcing **Jazzy**.

```bash
export PATH=/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/jazzy/setup.bash
cd /root/Elastic-Tracker/ros2_ws
colcon build --symlink-install
```


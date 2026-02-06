# Run on ROS 2 Humble (Ubuntu 22.04)

This repo is a ROS 2 workspace (`ros2_ws`). The commands below are tested on **Ubuntu 22.04 + ROS 2 Humble**.

## Prerequisites

- Install ROS 2 Humble (desktop recommended) so that `/opt/ros/humble` exists.
- Use **system Python** (avoid conda).

## Build

### 1) Source ROS 2 Humble

If you use **bash**:

```bash
source /opt/ros/humble/setup.bash
```

If you use **zsh**:

```zsh
source /opt/ros/humble/setup.zsh
```

### 2) (If needed) Provide missing ROS debs without sudo (pcl_ros)

Some systems don't have `pcl_ros` installed, but this workspace needs it. If you **can use sudo**, just install it:

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-pcl-ros
```

If you **cannot use sudo**, you can download+extract the `.deb` into a user overlay:

```bash
set -e
export WS="$HOME/catkin_ws/Elastic-Tracker/ros2_ws"
export OVERLAY_ROOT="$HOME/ros2_deb_overlay"
export APTROOT="$HOME/.cache/apt-user"

mkdir -p "$APTROOT/state/lists/partial" "$APTROOT/cache/archives/partial"
apt-get -o Dir::State="$APTROOT/state" -o Dir::State::lists="$APTROOT/state/lists" \
        -o Dir::Cache="$APTROOT/cache" -o Dir::Cache::archives="$APTROOT/cache/archives" \
        update

mkdir -p "$OVERLAY_ROOT/debs" "$OVERLAY_ROOT/extract"
cd "$OVERLAY_ROOT/debs"
apt-get -o Dir::State="$APTROOT/state" -o Dir::State::lists="$APTROOT/state/lists" \
        -o Dir::Cache="$APTROOT/cache" -o Dir::Cache::archives="$APTROOT/cache/archives" \
        download ros-humble-pcl-ros
dpkg-deb -x ros-humble-pcl-ros_*.deb "$OVERLAY_ROOT/extract"
```

After extraction, the overlay prefix is:

- `$OVERLAY_ROOT/extract/opt/ros/humble`

### 3) Build the workspace

```bash
set -e
cd "$HOME/catkin_ws/Elastic-Tracker/ros2_ws"

# If you used the no-sudo overlay (recommended to keep even if it exists but empty):
export OVERLAY="$HOME/ros2_deb_overlay/extract/opt/ros/humble"
export CMAKE_PREFIX_PATH="$OVERLAY:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$OVERLAY/lib:${LD_LIBRARY_PATH:-}"

rm -rf build install log
colcon build --symlink-install --merge-install
```

## Run (fake sim bringup)

```bash
set -e
source /opt/ros/humble/setup.bash
cd "$HOME/catkin_ws/Elastic-Tracker/ros2_ws"

# If you used the no-sudo overlay:
export OVERLAY="$HOME/ros2_deb_overlay/extract/opt/ros/humble"
export CMAKE_PREFIX_PATH="$OVERLAY:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$OVERLAY/lib:${LD_LIBRARY_PATH:-}"

source install/setup.bash

# Headless run (recommended):
ros2 launch elastic_tracker_bringup fake_sim.launch.py \
  enable_rviz:=false enable_local_sensing:=false
```

## Run with RViz2 (desktop)

`fake_sim.launch.py` supports RViz2, but it is **off by default**. On a machine with a real display, run:

```bash
ros2 launch elastic_tracker_bringup fake_sim.launch.py \
  enable_rviz:=true
```

Notes:

- `--show-args` only prints launch arguments, it does **not** start RViz or nodes.
- `use_xvfb` can be `true/false/auto` (default `auto`):
  - `auto`: uses Xvfb only when `DISPLAY` is unset/empty (typical headless/server case)
  - `false`: force a normal RViz window (desktop)
  - `true`: force `xvfb-run` (useful for remote/headless testing)
- Some RViz display plugins in `decomp_ros_utils` may be skipped on Humble if `rviz_rendering/objects/mesh_shape.hpp` is not shipped by your RViz build. This does **not** block core nodes from running.

## “Trigger” / start tracking (ROS2)

In ROS2, planning listens to topics inside the drone namespace:

- **Trigger**: `/drone0/triger` (yes, spelled `triger`) type `geometry_msgs/PoseStamped`
- **Land trigger**: `/drone0/land_triger` type `geometry_msgs/PoseStamped`
- Planning also requires a target odometry on **`/drone0/target`** (type `nav_msgs/Odometry`)

Example: publish a one-shot trigger pose:

```bash
ros2 topic pub --once /drone0/triger geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}"
```

If you don't have a target publisher running yet, you must also provide `/drone0/target` (message type `nav_msgs/Odometry`) from your detection/ekf pipeline, otherwise planning will not start tracking.

Useful launch args (see defaults):

```bash
ros2 launch elastic_tracker_bringup fake_sim.launch.py --show-args
```

## Notes / common issues

- `decomp_ros_utils` builds RViz display plugins. On some Humble builds, `rviz_rendering/objects/mesh_shape.hpp` is not shipped; in that case we **skip** building those RViz plugins (core nodes still build and run).


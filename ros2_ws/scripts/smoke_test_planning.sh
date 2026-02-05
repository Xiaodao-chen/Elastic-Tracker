#!/usr/bin/env bash
set -eo pipefail

export PATH=/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONHOME PYTHONPATH CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL CONDA_PYTHON_EXE CONDA_EXE _CONDA_EXE _CONDA_ROOT

cd /root/Elastic-Tracker/ros2_ws

# ROS setup files may reference unset tracing variables; avoid nounset during sourcing.
set +u
source /opt/ros/jazzy/setup.sh
source install/setup.sh
set -u

# Avoid FastDDS SHM issues in container/CI environments
export RMW_FASTRTPS_USE_SHM=0

echo "[smoke] killing leftovers..."
pkill -9 -f "ros2 launch elastic_tracker_bringup" 2>/dev/null || true
pkill -9 -f component_container_mt 2>/dev/null || true
pkill -9 -f mockamap_node 2>/dev/null || true
pkill -9 -f traj_server_node 2>/dev/null || true
pkill -9 -f mapping_vis_node 2>/dev/null || true
pkill -9 -f odom_visualization_node 2>/dev/null || true
sleep 1

LOG=/tmp/planning_smoke.log
rm -f "$LOG"

echo "[smoke] starting bringup..."
ros2 launch -n elastic_tracker_bringup fake_sim.launch.py \
  planning_params:=/root/Elastic-Tracker/ros2_ws/install/share/elastic_tracker_bringup/config/planning.yaml \
  enable_rviz:=false >"$LOG" 2>&1 &
LPID=$!

cleanup() {
  kill "$LPID" 2>/dev/null || true
  sleep 1
  pkill -P "$LPID" 2>/dev/null || true
}
trap cleanup EXIT

sleep 6

echo "[smoke] topics check..."
ros2 topic list | egrep '^/drone0/(gridmap_inflate|odom|triger|target|planning/polyhedra|trajectory|replanState)$' >/dev/null

echo "[smoke] publish trigger..."
ros2 topic pub -1 /drone0/triger geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}' >/dev/null

echo "[smoke] publish target..."
ros2 topic pub -r 10 -t 40 /drone0/target nav_msgs/msg/Odometry \
  '{header: {frame_id: world}, pose: {pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, twist: {twist: {linear: {x: 0.0, y: 0.0, z: 0.0}}}}' >/dev/null

echo "[smoke] expect polyhedra + trajectory + replanState..."
ros2 topic echo /drone0/planning/polyhedra decomp_ros_msgs/msg/PolyhedronArray --once --timeout 10 >/dev/null
ros2 topic echo /drone0/trajectory quadrotor_msgs/msg/PolyTraj --once --timeout 10 >/dev/null
ros2 topic echo /drone0/replanState quadrotor_msgs/msg/ReplanState --once --timeout 10 >/dev/null

echo "[smoke] OK"


#!/usr/bin/env bash
set -eo pipefail

export PATH=/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONHOME PYTHONPATH CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL CONDA_PYTHON_EXE CONDA_EXE _CONDA_EXE _CONDA_ROOT

cd /root/Elastic-Tracker/ros2_ws

# Avoid nounset issues while sourcing setup files
set +u
source /opt/ros/jazzy/setup.sh
source install/setup.sh
set -u

# Avoid FastDDS SHM issues in container/CI environments
export RMW_FASTRTPS_USE_SHM=0

echo "[full-chain] killing leftovers..."
pkill -9 -f "ros2 launch elastic_tracker_bringup" 2>/dev/null || true
pkill -9 -f component_container_mt 2>/dev/null || true
pkill -9 -f mockamap_node 2>/dev/null || true
pkill -9 -f traj_server_node 2>/dev/null || true
pkill -9 -f mapping_vis_node 2>/dev/null || true
pkill -9 -f odom_visualization_node 2>/dev/null || true
pkill -9 -f pcl_render_node 2>/dev/null || true
sleep 1

LOG=/tmp/full_chain_smoke.log
rm -f "$LOG"

echo "[full-chain] starting bringup (planning=normal, local_sensing=on)..."
ros2 launch -n elastic_tracker_bringup fake_sim.launch.py \
  planning_params:=/root/Elastic-Tracker/ros2_ws/install/share/elastic_tracker_bringup/config/planning.yaml \
  enable_local_sensing:=true enable_rviz:=false >"$LOG" 2>&1 &
LPID=$!
trap "kill $LPID 2>/dev/null || true; sleep 1; pkill -P $LPID 2>/dev/null || true" EXIT

sleep 8

echo "[full-chain] basic topics..."
ros2 topic list | egrep '^/drone0/(odom|gridmap_inflate|trajectory|replanState|planning/polyhedra|triger|target|position_cmd|so3cmd|depth|local_cloud)$' >/dev/null

echo "[full-chain] wait one odom + gridmap..."
ros2 topic echo /drone0/odom nav_msgs/msg/Odometry --once --timeout 10 >/dev/null
ros2 topic echo /drone0/gridmap_inflate quadrotor_msgs/msg/OccMap3d --once --timeout 10 >/dev/null

echo "[full-chain] publish trigger..."
ros2 topic pub -1 /drone0/triger geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}' >/dev/null

echo "[full-chain] publish target..."
ros2 topic pub -r 10 -t 60 /drone0/target nav_msgs/msg/Odometry \
  '{header: {frame_id: world}, pose: {pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, twist: {twist: {linear: {x: 0.0, y: 0.0, z: 0.0}}}}' >/dev/null

echo "[full-chain] expect planner outputs..."
ros2 topic echo /drone0/planning/polyhedra decomp_ros_msgs/msg/PolyhedronArray --once --timeout 12 >/dev/null
ros2 topic echo /drone0/trajectory quadrotor_msgs/msg/PolyTraj --once --timeout 12 >/dev/null
ros2 topic echo /drone0/replanState quadrotor_msgs/msg/ReplanState --once --timeout 12 >/dev/null

echo "[full-chain] expect controller/sim outputs..."
ros2 topic echo /drone0/position_cmd quadrotor_msgs/msg/PositionCommand --once --timeout 12 >/dev/null
ros2 topic echo /drone0/so3cmd quadrotor_msgs/msg/SO3Command --once --timeout 12 >/dev/null

echo "[full-chain] expect local_sensing outputs..."
ros2 topic echo /drone0/depth sensor_msgs/msg/Image --once --timeout 12 >/dev/null
ros2 topic echo /drone0/local_cloud sensor_msgs/msg/PointCloud2 --once --timeout 12 >/dev/null

echo "[full-chain] OK"


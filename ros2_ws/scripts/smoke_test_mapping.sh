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

export RMW_FASTRTPS_USE_SHM=0

echo "[smoke(mapping)] killing leftovers..."
pkill -9 -f "ros2 launch elastic_tracker_bringup" 2>/dev/null || true
pkill -9 -f component_container_mt 2>/dev/null || true
pkill -9 -f mockamap_node 2>/dev/null || true
pkill -9 -f traj_server_node 2>/dev/null || true
pkill -9 -f mapping_vis_node 2>/dev/null || true
pkill -9 -f odom_visualization_node 2>/dev/null || true
sleep 1

LOG=/tmp/mapping_smoke.log
rm -f "$LOG"

echo "[smoke(mapping)] starting bringup (planning fake ok)..."
ros2 launch -n elastic_tracker_bringup fake_sim.launch.py \
  enable_rviz:=false >"$LOG" 2>&1 &
LPID=$!
trap "kill $LPID 2>/dev/null || true; sleep 1; pkill -P $LPID 2>/dev/null || true" EXIT

sleep 6

echo "[smoke(mapping)] expect gridmap_inflate..."
ros2 topic echo /drone0/gridmap_inflate quadrotor_msgs/msg/OccMap3d --once --timeout 10 >/dev/null

echo "[smoke(mapping)] expect mapping vis topics..."
# mapping_vis node publishes PointCloud2 converted from occmap
ros2 topic echo /drone0/mapping_vis/vs_gridmap sensor_msgs/msg/PointCloud2 --once --timeout 10 >/dev/null

echo "[smoke(mapping)] OK"


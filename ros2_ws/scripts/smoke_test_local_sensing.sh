#!/usr/bin/env bash
set -eo pipefail

export PATH=/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONHOME PYTHONPATH CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL CONDA_PYTHON_EXE CONDA_EXE _CONDA_EXE _CONDA_ROOT

cd /root/Elastic-Tracker/ros2_ws

set +u
source /opt/ros/jazzy/setup.sh
source install/setup.sh
set -u

export RMW_FASTRTPS_USE_SHM=0

echo "[smoke(local_sensing)] killing leftovers..."
pkill -9 -f "ros2 launch elastic_tracker_bringup" 2>/dev/null || true
pkill -9 -f component_container_mt 2>/dev/null || true
pkill -9 -f mockamap_node 2>/dev/null || true
pkill -9 -f traj_server_node 2>/dev/null || true
pkill -9 -f mapping_vis_node 2>/dev/null || true
pkill -9 -f odom_visualization_node 2>/dev/null || true
pkill -9 -f pcl_render_node 2>/dev/null || true
sleep 1

LOG=/tmp/local_sensing_smoke.log
rm -f "$LOG"

echo "[smoke(local_sensing)] starting bringup with enable_local_sensing..."
ros2 launch -n elastic_tracker_bringup fake_sim.launch.py \
  enable_local_sensing:=true enable_rviz:=false >"$LOG" 2>&1 &
LPID=$!
trap "kill $LPID 2>/dev/null || true; sleep 1; pkill -P $LPID 2>/dev/null || true" EXIT

sleep 8

echo "[smoke(local_sensing)] expect depth + local_cloud..."
ros2 topic echo /drone0/depth sensor_msgs/msg/Image --once --timeout 12 >/dev/null
ros2 topic echo /drone0/local_cloud sensor_msgs/msg/PointCloud2 --once --timeout 12 >/dev/null

echo "[smoke(local_sensing)] OK"


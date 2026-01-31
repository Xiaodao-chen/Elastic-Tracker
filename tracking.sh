#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

try_source() {
  local f="$1"
  if [[ -f "$f" ]]; then
    # shellcheck disable=SC1090
    source "$f"
    return 0
  fi
  return 1
}

ensure_ros_env() {
  if command -v roslaunch >/dev/null 2>&1; then
    return 0
  fi

  # Try sourcing system ROS setup if available.
  if [[ -d /opt/ros ]]; then
    local setup
    for setup in /opt/ros/*/setup.bash /opt/ros/*/setup.zsh; do
      [[ -f "$setup" ]] || continue
      # shellcheck disable=SC1090
      source "$setup"
      break
    done
  fi

  if ! command -v roslaunch >/dev/null 2>&1; then
    echo "[tracking.sh] 未找到 roslaunch。请先 source 你的 ROS 环境（如 /opt/ros/<distro>/setup.bash）以及工作空间 devel/setup.*" >&2
    exit 1
  fi
}

ensure_ws_env() {
  # Source catkin workspace env if present (prefer bash).
  try_source "$ROOT_DIR/devel/setup.bash" || true
  try_source "$ROOT_DIR/devel/setup.zsh" || true
}

cleanup() {
  # Gracefully stop background roslaunch processes if still running.
  local pid
  for pid in "${PIDS[@]:-}"; do
    if [[ -n "${pid:-}" ]] && kill -0 "$pid" >/dev/null 2>&1; then
      kill -INT "$pid" >/dev/null 2>&1 || true
    fi
  done
}

PIDS=()
trap cleanup EXIT INT TERM

ensure_ros_env
ensure_ws_env

echo "[tracking.sh] 启动 RViz 可视化：roslaunch mapping rviz_sim.launch"
roslaunch mapping rviz_sim.launch &
PIDS+=("$!")

sleep 2

echo "[tracking.sh] 启动 fake target：roslaunch planning fake_target.launch"
roslaunch planning fake_target.launch &
PIDS+=("$!")

sleep 2

echo "[tracking.sh] 启动 elastic tracker：roslaunch planning simulation1.launch"
roslaunch planning simulation1.launch

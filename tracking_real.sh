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
    echo "[tracking_real.sh] 未找到 roslaunch，请先 source ROS 环境。" >&2
    exit 1
  fi
}

ensure_ws_env() {
  try_source "$ROOT_DIR/devel/setup.bash" || true
  try_source "$ROOT_DIR/devel/setup.zsh" || true
}

ensure_ros_env
ensure_ws_env

# === 可改参数（也可用环境变量覆盖） ===
: "${UAV_ODOM_TOPIC:=/ekf/ekf_odom}"
: "${TARGET_ODOM_TOPIC:=/target_odom}"
: "${DEPTH_TOPIC:=/camera/depth/image_rect_raw}"
: "${CMD_VEL_TOPIC:=/mavros/setpoint_velocity/cmd_vel}"
: "${WITH_MAPPING:=true}"
: "${RHO_TRACKING:=1000.0}"
: "${RHOS_VISIBILITY:=10000.0}"

echo "[tracking_real.sh] UAV_ODOM_TOPIC=${UAV_ODOM_TOPIC}"
echo "[tracking_real.sh] TARGET_ODOM_TOPIC=${TARGET_ODOM_TOPIC}"
echo "[tracking_real.sh] DEPTH_TOPIC=${DEPTH_TOPIC}"
echo "[tracking_real.sh] CMD_VEL_TOPIC=${CMD_VEL_TOPIC}"
echo "[tracking_real.sh] WITH_MAPPING=${WITH_MAPPING}"

exec roslaunch planning real_tracking.launch \
  with_mapping:="${WITH_MAPPING}" \
  uav_odom_topic:="${UAV_ODOM_TOPIC}" \
  target_odom_topic:="${TARGET_ODOM_TOPIC}" \
  depth_topic:="${DEPTH_TOPIC}" \
  cmd_vel_topic:="${CMD_VEL_TOPIC}" \
  rhoTracking_:="${RHO_TRACKING}" \
  rhosVisibility_:="${RHOS_VISIBILITY}"


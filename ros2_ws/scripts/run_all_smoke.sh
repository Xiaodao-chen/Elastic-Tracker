#!/usr/bin/env bash
set -eo pipefail

cd /root/Elastic-Tracker/ros2_ws
chmod +x scripts/smoke_test_*.sh

echo "[run] traj_opt"
bash scripts/smoke_test_traj_opt.sh

echo "[run] mapping"
bash scripts/smoke_test_mapping.sh

echo "[run] local_sensing"
bash scripts/smoke_test_local_sensing.sh

echo "[run] planning"
bash scripts/smoke_test_planning.sh

echo "[run] full_chain"
bash scripts/smoke_test_full_chain.sh

echo "[run] ALL OK"


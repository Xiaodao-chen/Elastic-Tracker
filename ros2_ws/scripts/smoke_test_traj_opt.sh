#!/usr/bin/env bash
set -eo pipefail

export PATH=/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONHOME PYTHONPATH CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL CONDA_PYTHON_EXE CONDA_EXE _CONDA_EXE _CONDA_ROOT

cd /root/Elastic-Tracker/ros2_ws

set +u
source /opt/ros/jazzy/setup.sh
source install/setup.sh
set -u

echo "[smoke(traj_opt)] headers present?"
test -f install/include/traj_opt/traj_opt.h
test -f install/include/traj_opt/poly_traj_utils.hpp

echo "[smoke(traj_opt)] shared library present?"
test -f install/lib/libtraj_opt_core.so

echo "[smoke(traj_opt)] CMake package present?"
test -f install/share/traj_opt/cmake/traj_optConfig.cmake
test -f install/share/traj_opt_core/cmake/traj_opt_coreConfig.cmake

echo "[smoke(traj_opt)] OK"


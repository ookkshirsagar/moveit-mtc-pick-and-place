#!/usr/bin/env bash
# Start the Isaac Sim side of the demo (hardware:=isaac).
#
#   source install/setup.bash                       # the colcon workspace
#   ./src/moveit-mtc-pick-and-place/isaac/run_scene.sh [--side left] [--headless] ...
#
# ISAAC_SIM (default ~/isaacsim) points at the Isaac Sim 5.x/6.x install. Every argument is passed on to
# pick_place_scene.py; the URDF is expanded here from the workspace xacro with hardware:=isaac.
set -euo pipefail

ISAAC_SIM=${ISAAC_SIM:-$HOME/isaacsim}
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
: "${AMENT_PREFIX_PATH:?source /opt/ros/jazzy/setup.bash and the workspace install/setup.bash first}"

SIDE=left
for ((i = 1; i <= $#; i++)); do
  [[ ${!i} == --side ]] && { j=$((i + 1)); SIDE=${!j}; }
done

TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
URDF=$TMP/ur5e_rh56dfx_${SIDE}.urdf                  # the file name also names the USD cache folder
xacro "$(ros2 pkg prefix --share ur5e_rh56dfx_moveit_config)/config/ur5e_rh56dfx_system.xacro" \
  side:="$SIDE" prefix:="${SIDE}_" hardware:=isaac > "$URDF"

# Isaac ships its own ROS 2 Jazzy libraries; keep the system Python packages out of its interpreter.
env -u PYTHONPATH -u LD_LIBRARY_PATH \
  AMENT_PREFIX_PATH="$AMENT_PREFIX_PATH" \
  ROS_DISTRO=jazzy RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  LD_LIBRARY_PATH="$ISAAC_SIM/exts/isaacsim.ros2.core/jazzy/lib" \
  "$ISAAC_SIM/python.sh" "$HERE/pick_place_scene.py" --urdf "$URDF" --usd-dir "$TMP/usd" "$@"

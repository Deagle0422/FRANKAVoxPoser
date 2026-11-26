#!/usr/bin/env bash

set -eo pipefail

TRACE_LEVEL="${FR3_MOVEIT_INTERNAL_TRACE:-samples}"
OVERLAY_SETUP="/moveit_trace_ws/install/setup.bash"

if [[ ! -f "${OVERLAY_SETUP}" ]]; then
  echo "MoveIt trace overlay is not built: ${OVERLAY_SETUP}" >&2
  echo "Build the overlay before starting the Gazebo trace launch." >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
source "${OVERLAY_SETUP}"

echo "Starting Gazebo with FR3_MOVEIT_INTERNAL_TRACE=${TRACE_LEVEL}"
echo "This command launches fr3_moveit_gazebo.launch.py only; it does not connect to a real FR3."

export FR3_MOVEIT_INTERNAL_TRACE="${TRACE_LEVEL}"
exec ros2 launch franka_fr3_moveit_config fr3_moveit_gazebo.launch.py "$@"

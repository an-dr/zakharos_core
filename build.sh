#!/bin/bash
set -e # exit on error
WS_DIR=$(dirname $(readlink -f "$0"))
SCRIPT_NAME=$(basename "$0")
function log { echo "- $1 [$SCRIPT_NAME]" ;}
# ----------------------------------------------------------------------------

log "Workspace dir is $WS_DIR"

if [ -z "$ROS_DISTRO" ]
then
    log "ROS is not found. Trying to setup the environment"
    source /opt/ros/noetic/setup.bash
else
    log "ROS is found!"
fi

log "Build workspace..."
catkin build --workspace "$WS_DIR"
log "Build complete"

log "[   DONE   ]"

# ----------------------------------------------------------------------------
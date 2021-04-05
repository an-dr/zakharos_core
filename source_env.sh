#!/bin/bash
SCRIPT_DIR=$(cd $(dirname "${BASH_SOURCE[0]}") && pwd)
SCRIPT_NAME="${BASH_SOURCE[0]}"
SCRIPT_PATH="$SCRIPT_DIR/$SCRIPT_NAME"
function log { echo "- $1 [$SCRIPT_NAME]" ;}
# ----------------------------------------------------------------------------

export ROSCONSOLE_FORMAT='${severity}: ${message} | ${file}:${line}'

# 1. Check if sourced
if [ -n "${BASH_SOURCE}" ] && [ "${BASH_SOURCE[0]}" = "${0}" ] ; then
    echo -e "This script should be sourced, not executed: \n. $SCRIPT_PATH"
    exit 1
fi


# 2. Set ros setup.bash
ROS_SETUP_BASH="/opt/ros/noetic/setup.bash"


# 3. Standard ZAKHAR variables
export ZAKHAROS_CORE_PATH="$SCRIPT_DIR"
export ZAKHAROS_SETUP_SOURCE="$ZAKHAROS_CORE_PATH/devel/setup.bash"


# 4. Source ROS environment
if [ -f "$ZAKHAROS_SETUP_SOURCE" ]; then
    source $ZAKHAROS_SETUP_SOURCE
else
    source $ROS_SETUP_BASH
fi

#!/bin/bash
set -e # exit on error
WS_DIR=$(dirname $(readlink -f "$0"))
SCRIPT_NAME=$(basename "$0")
function log { echo "- $1 [$SCRIPT_NAME]" ;}
# ----------------------------------------------------------------------------


log "Applying an execution parameter to *.py"
find $WS_DIR/src/ -name "*.py" -exec chmod -c -x {} \;
find $WS_DIR/src/ -wholename "**/nodes/*.py" -exec chmod -c +x {} \;
find $WS_DIR/src/ -wholename "**/scripts/*.py" -exec chmod -c +x {} \;
log "Applying an execution parameter to *.sh"
find $WS_DIR/src/ -name "*.sh" -exec chmod -c +x {} \;
find $WS_DIR/scripts/ -name "*.sh" -exec chmod -c +x {} \;

log "[   DONE   ]"
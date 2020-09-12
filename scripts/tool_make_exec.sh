#!/bin/bash
set -e # exit on error
SCRIPT_ROOT=$(dirname $(readlink -f "$0"))
SCRIPT_NAME=$(basename "$0")
WS_DIR=$(dirname "$SCRIPT_ROOT")
function log { echo "- $1 [$(basename "$0")]" ;}
# ----------------------------------------------------------------------------

log "Applying an execution parameter to *.py"
find $WS_DIR/src/ -name "*.py" -exec chmod -c -x {} \;
find $WS_DIR/src/ -wholename "**/nodes/*.py" -exec chmod -c +x {} \;
find $WS_DIR/src/ -wholename "**/scripts/*.py" -exec chmod -c +x {} \;
log "Applying an execution parameter to *.sh"
find $WS_DIR/src/ -name "*.sh" -exec chmod -c +x {} \;
find $WS_DIR/scripts/ -name "*.sh" -exec chmod -c +x {} \;

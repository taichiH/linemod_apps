#!/bin/bash
set -e

# setup ros environment
if [ -z "${SETUP}" ]; then
    source "/ros_ws/devel/setup.bash"
else
    source $SETUP
fi

exec "$@"

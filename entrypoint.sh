#!/bin/bash
set -e

# setup ros environment
source "/root/ros/devel/setup.bash"

exec "$@"
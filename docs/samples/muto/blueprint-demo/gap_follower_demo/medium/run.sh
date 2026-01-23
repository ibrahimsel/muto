#!/bin/bash
#
# Gap Follower - MEDIUM variant
# MAX_SPEED=1.0, MIN_SPEED=0.5
#

set -e

echo "=== Gap Follower MEDIUM Variant ==="
echo "MAX_SPEED: 1.0 m/s"
echo "MIN_SPEED: 0.5 m/s"

# Source ROS environment
source /opt/ros/humble/setup.bash

# Build the workspace if not already built
if [ ! -d "install" ]; then
    echo "Building workspace..."
    colcon build --symlink-install
fi

# Source the workspace
source install/setup.bash

# Export parameters as environment variables
export GAP_FOLLOWER_MAX_SPEED=1.0
export GAP_FOLLOWER_MIN_SPEED=0.5
export GAP_FOLLOWER_CAR_WIDTH=0.2032

echo "Starting gap_follower node (medium)..."
ros2 run gap_follower gap_follower

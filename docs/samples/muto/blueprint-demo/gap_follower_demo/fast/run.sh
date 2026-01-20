#!/bin/bash
#
# Gap Follower - FAST variant (INTENTIONALLY FAILS FOR DEMO)
# This variant is designed to fail to demonstrate rollback functionality.
#

set -e

echo "=== Gap Follower FAST Variant ==="
echo "MAX_SPEED: 2.0 m/s"
echo "MIN_SPEED: 1.5 m/s"
echo ""
echo "WARNING: This variant will fail intentionally for rollback demonstration"

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
export GAP_FOLLOWER_MAX_SPEED=2.0
export GAP_FOLLOWER_MIN_SPEED=1.5

echo "Starting gap_follower node (fast)..."
echo "Simulating startup..."
sleep 3

# INTENTIONAL FAILURE FOR ROLLBACK DEMO
echo ""
echo "ERROR: Simulated failure for rollback demonstration"
echo "The fast variant has encountered a critical error."
echo "Muto should now trigger automatic rollback to the previous version."
exit 1

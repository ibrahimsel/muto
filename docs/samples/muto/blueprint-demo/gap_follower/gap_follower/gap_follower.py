#
# Copyright (c) 2025 Composiv.ai
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
#
# SPDX-License-Identifier: EPL-2.0
#
# Contributors:
#   Composiv.ai - initial API and implementation
#

import rclpy, numpy as np, math, os
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

MAX_SPEED = 1.5
MIN_SPEED = 1.0
ANGLE_GAIN = 1.7
SAFE_GAP = 2.0          # m
BUBBLE_RADIUS = 0.3     # m
FOV = math.radians(270) # the lidar in gym is 270 degrees

def best_gap(ranges, angle_min, angle_inc):
    arr = np.array(ranges)
    arr[arr < BUBBLE_RADIUS] = 0.0
    mask = arr > SAFE_GAP
    if not mask.any():
        return 0.0
    segs = np.where(np.diff(mask.astype(int)) != 0)[0] + 1
    indices = np.split(np.arange(arr.size), segs)
    gaps = [idx for idx in indices if mask[idx].any()]
    best = max(gaps, key=len)
    mid = best[len(best)//2]
    return angle_min + mid * angle_inc

class GapFollower(Node):
    def __init__(self):
        hostname = os.uname()[1]
        super().__init__(f'{hostname}_gap_follower')
        self.drive_pub = self.create_publisher(AckermannDriveStamped, f'/{hostname}/drive', 1)
        self.create_subscription(LaserScan, f'/{hostname}/scan', self.cb, 1)

    def cb(self, scan):
        steer = best_gap(scan.ranges, scan.angle_min, scan.angle_increment)
        speed = max(MIN_SPEED, MAX_SPEED * (1 - abs(steer)/ (FOV/2)))
        msg = AckermannDriveStamped()
        msg.header.stamp = scan.header.stamp
        msg.header.frame_id = f"{self.ns}/base_link"
        msg.drive.steering_angle = steer * ANGLE_GAIN
        msg.drive.speed = speed
        self.drive_pub.publish(msg)

def main():
    rclpy.init()
    g = GapFollower()
    rclpy.spin(g)
    g.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

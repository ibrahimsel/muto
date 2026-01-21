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
# Optimized for racing on Spielberg (Red Bull Ring) track
# Track: ~1.7-2.9m wide, sweeping curves, long straights, 0.058m/pixel resolution
# Vehicle: 0.2m wide, 0.33m wheelbase, collision threshold 0.21m
#

import rclpy
import numpy as np
import math
import os
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

# Read parameters from environment variables with defaults optimized for Spielberg racing
MAX_SPEED = float(os.environ.get('GAP_FOLLOWER_MAX_SPEED', '3.0'))
MIN_SPEED = float(os.environ.get('GAP_FOLLOWER_MIN_SPEED', '1.0'))
ANGLE_GAIN = float(os.environ.get('GAP_FOLLOWER_ANGLE_GAIN', '1.2'))
# For wider racing track, moderate safe_gap
SAFE_GAP = float(os.environ.get('GAP_FOLLOWER_SAFE_GAP', '0.8'))
# Bubble just larger than collision threshold (0.21m)
BUBBLE_RADIUS = float(os.environ.get('GAP_FOLLOWER_BUBBLE_RADIUS', '0.25'))
# Emergency brake distance
FRONT_DISTANCE_THRESHOLD = float(os.environ.get('GAP_FOLLOWER_FRONT_THRESHOLD', '0.4'))
# Start slowing down distance - longer for higher speeds
SLOWDOWN_DISTANCE = float(os.environ.get('GAP_FOLLOWER_SLOWDOWN_DIST', '2.0'))
# Steering smoothing factor (0=no smoothing, 1=max smoothing) - smoother for racing
STEER_SMOOTH = float(os.environ.get('GAP_FOLLOWER_STEER_SMOOTH', '0.4'))
# Car width for disparity extender
CAR_WIDTH = float(os.environ.get('GAP_FOLLOWER_CAR_WIDTH', '0.25'))

FOV = math.radians(270)  # the lidar in gym is 270 degrees
HALF_FOV = FOV / 2


def preprocess_lidar(ranges):
    """Clean up lidar data - replace invalid readings."""
    arr = np.array(ranges, dtype=np.float64)
    # Replace inf/nan with max range (30m)
    arr = np.where(np.isfinite(arr), arr, 30.0)
    # Clip to reasonable range
    arr = np.clip(arr, 0.0, 30.0)
    return arr


def apply_safety_bubble(ranges, closest_idx, bubble_radius, angle_inc):
    """Zero out points around the closest obstacle."""
    if closest_idx < 0:
        return ranges

    result = ranges.copy()
    n = len(ranges)

    # Calculate how many indices the bubble covers
    bubble_indices = int(bubble_radius / (ranges[closest_idx] * angle_inc + 0.001)) + 1
    bubble_indices = min(bubble_indices, 50)  # Cap to prevent excessive zeroing

    start = max(0, closest_idx - bubble_indices)
    end = min(n, closest_idx + bubble_indices + 1)
    result[start:end] = 0.0

    return result


def apply_disparity_extender(ranges, angle_inc, car_width):
    """Extend obstacles at disparity points to prevent corner clipping."""
    extended = ranges.copy()
    n = len(ranges)

    # Disparity threshold - detect significant depth changes
    disparity_threshold = 0.3  # meters

    for i in range(1, n):
        diff = ranges[i] - ranges[i-1]
        if abs(diff) > disparity_threshold:
            # Found a disparity - extend the closer point
            closer_dist = min(ranges[i], ranges[i-1])
            if closer_dist > 0.1:
                # Calculate extension based on car width and distance
                extend_angle = math.atan2(car_width, closer_dist)
                extend_indices = int(extend_angle / angle_inc) + 1

                # Extend into the gap (towards the farther reading)
                if diff > 0:  # ranges[i] is farther, extend forward
                    for j in range(i, min(n, i + extend_indices)):
                        extended[j] = min(extended[j], closer_dist)
                else:  # ranges[i-1] is farther, extend backward
                    for j in range(max(0, i - extend_indices), i):
                        extended[j] = min(extended[j], closer_dist)

    return extended


def find_best_gap(ranges, angle_min, angle_inc, safe_gap):
    """
    Find the best gap using the Follow the Gap method.
    Optimized for tight corridors - finds the deepest point in the widest safe gap.
    """
    n = len(ranges)

    # Find all points that exceed safe_gap threshold
    safe_mask = ranges > safe_gap

    if not safe_mask.any():
        # No safe gaps - head towards the farthest visible point
        best_idx = np.argmax(ranges)
        return angle_min + best_idx * angle_inc, ranges[best_idx]

    # Find contiguous safe regions
    # Pad with False to detect edges properly
    padded = np.concatenate([[False], safe_mask, [False]])
    diff = np.diff(padded.astype(int))
    starts = np.where(diff == 1)[0]
    ends = np.where(diff == -1)[0]

    if len(starts) == 0:
        best_idx = np.argmax(ranges)
        return angle_min + best_idx * angle_inc, ranges[best_idx]

    # Score each gap by: width * max_depth
    # This prefers wide gaps but also considers depth
    best_score = -1
    best_gap_start = 0
    best_gap_end = 0

    for start, end in zip(starts, ends):
        width = end - start
        gap_ranges = ranges[start:end]
        max_depth = np.max(gap_ranges) if len(gap_ranges) > 0 else 0

        # Score: prioritize width but reward depth
        score = width * (1.0 + max_depth * 0.5)

        if score > best_score:
            best_score = score
            best_gap_start = start
            best_gap_end = end

    if best_gap_end <= best_gap_start:
        best_idx = np.argmax(ranges)
        return angle_min + best_idx * angle_inc, ranges[best_idx]

    # Find the deepest point in the best gap
    gap_ranges = ranges[best_gap_start:best_gap_end]
    deepest_in_gap = np.argmax(gap_ranges)
    deepest_idx = best_gap_start + deepest_in_gap

    # Also compute center of gap
    center_idx = (best_gap_start + best_gap_end) // 2

    # Blend: favor the deepest point but bias towards center for stability
    # More aggressive = favor deepest point more
    blend = 0.6  # 60% deepest, 40% center
    target_idx = int(blend * deepest_idx + (1 - blend) * center_idx)
    target_idx = max(best_gap_start, min(best_gap_end - 1, target_idx))

    target_angle = angle_min + target_idx * angle_inc
    return target_angle, ranges[target_idx]


def get_front_distance(ranges, angle_inc):
    """Get minimum distance in a narrow front cone."""
    n = len(ranges)
    center = n // 2

    # Narrow cone: +/- 15 degrees for tight corridor racing
    cone_angle = math.radians(15)
    cone_indices = int(cone_angle / angle_inc)

    start = max(0, center - cone_indices)
    end = min(n, center + cone_indices + 1)

    front = ranges[start:end]
    valid = front[front > 0.1]

    if len(valid) == 0:
        return 30.0
    return np.min(valid)


def get_side_distances(ranges, angle_inc):
    """Get distances to left and right sides for wall following bias."""
    n = len(ranges)

    # Left side: around 90 degrees from center
    left_angle = math.radians(90)
    left_idx = int((HALF_FOV + left_angle) / angle_inc)
    left_idx = min(n - 1, max(0, left_idx))

    # Right side: around -90 degrees from center
    right_angle = math.radians(-90)
    right_idx = int((HALF_FOV + right_angle) / angle_inc)
    right_idx = min(n - 1, max(0, right_idx))

    # Average a few points around each side
    window = 20
    left_start = max(0, left_idx - window)
    left_end = min(n, left_idx + window)
    right_start = max(0, right_idx - window)
    right_end = min(n, right_idx + window)

    left_dist = np.mean(ranges[left_start:left_end])
    right_dist = np.mean(ranges[right_start:right_end])

    return left_dist, right_dist


class GapFollower(Node):
    def __init__(self):
        self.hostname = os.uname()[1]
        super().__init__(f'{self.hostname}_gap_follower')

        # Log the parameters being used
        self.get_logger().info(
            f'Gap Follower [RACING MODE] starting with: '
            f'MAX_SPEED={MAX_SPEED}, MIN_SPEED={MIN_SPEED}, '
            f'SAFE_GAP={SAFE_GAP}, BUBBLE={BUBBLE_RADIUS}, GAIN={ANGLE_GAIN}'
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, f'/{self.hostname}/drive', 1
        )
        self.create_subscription(LaserScan, f'/{self.hostname}/scan', self.cb, 1)

        # State for smoothing
        self.last_steer = 0.0
        self.last_speed = MIN_SPEED

    def cb(self, scan):
        # Preprocess lidar data
        ranges = preprocess_lidar(scan.ranges)
        angle_inc = scan.angle_increment

        # Find closest point
        closest_dist = np.min(ranges[ranges > 0.1]) if np.any(ranges > 0.1) else 30.0
        closest_idx = np.argmin(np.where(ranges > 0.1, ranges, 100.0))

        # Apply safety bubble around closest point
        ranges = apply_safety_bubble(ranges, closest_idx, BUBBLE_RADIUS, angle_inc)

        # Apply disparity extender
        ranges = apply_disparity_extender(ranges, angle_inc, CAR_WIDTH)

        # Get front distance
        front_dist = get_front_distance(ranges, angle_inc)

        # Emergency stop if about to collide
        if front_dist < FRONT_DISTANCE_THRESHOLD:
            # Try to steer away from obstacle instead of stopping
            left_dist, right_dist = get_side_distances(
                preprocess_lidar(scan.ranges), angle_inc
            )
            if left_dist > right_dist:
                emergency_steer = 0.4  # Turn left
            else:
                emergency_steer = -0.4  # Turn right

            # Very slow speed during emergency
            self.publish_drive(scan.header.stamp, emergency_steer * ANGLE_GAIN, MIN_SPEED * 0.5)
            return

        # Find best gap
        steer_angle, gap_depth = find_best_gap(
            ranges, scan.angle_min, angle_inc, SAFE_GAP
        )

        # Smooth steering - less smoothing for more responsive racing
        steer_angle = (1 - STEER_SMOOTH) * steer_angle + STEER_SMOOTH * self.last_steer
        self.last_steer = steer_angle

        # Calculate speed based on multiple factors
        # 1. Steering magnitude - slow down for turns
        steer_magnitude = abs(steer_angle)
        steer_factor = 1.0 - min(0.7, steer_magnitude / (HALF_FOV * 0.5))

        # 2. Front distance - slow down approaching walls
        if front_dist < SLOWDOWN_DISTANCE:
            dist_ratio = (front_dist - FRONT_DISTANCE_THRESHOLD) / (SLOWDOWN_DISTANCE - FRONT_DISTANCE_THRESHOLD)
            dist_factor = max(0.2, min(1.0, dist_ratio))
        else:
            dist_factor = 1.0

        # 3. Gap depth - more confident with deeper gaps
        depth_factor = min(1.0, gap_depth / 2.0)

        # Combine factors - use minimum for safety but weighted
        speed_factor = min(steer_factor, dist_factor) * (0.7 + 0.3 * depth_factor)
        target_speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * speed_factor

        # Smooth speed changes slightly
        speed = 0.7 * target_speed + 0.3 * self.last_speed
        self.last_speed = speed

        # Apply steering gain
        final_steer = steer_angle * ANGLE_GAIN

        # Clamp steering to physical limits
        max_steer = 0.4189  # ~24 degrees, typical for F1TENTH
        final_steer = max(-max_steer, min(max_steer, final_steer))

        self.publish_drive(scan.header.stamp, final_steer, speed)

    def publish_drive(self, stamp, steer, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = f"{self.hostname}/base_link"
        msg.drive.steering_angle = float(steer)
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)


def main():
    rclpy.init()
    g = GapFollower()
    rclpy.spin(g)
    g.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

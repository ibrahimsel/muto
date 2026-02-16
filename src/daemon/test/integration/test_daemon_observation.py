#!/usr/bin/env python3
# Copyright (c) 2025 Composiv.ai
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
#
# SPDX-License-Identifier: EPL-2.0

"""
launch_testing integration test for muto_daemon graph observation.

Launches muto_daemon with demo_nodes_cpp talker/listener and verifies:
1. GraphSnapshot is published on /muto/graph_state with discovered nodes
2. After publishing DesiredState, drift detection reports convergence
"""

import time
import unittest

import launch
import launch_testing
import launch_testing.actions
from launch_ros.actions import Node

import rclpy
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from muto_msgs.msg import DesiredState, GraphSnapshot, NodeState


def generate_test_description():
    return launch.LaunchDescription([
        Node(
            package='muto_daemon',
            executable='muto_daemon',
            name='muto_daemon',
            parameters=[{
                'graph_observer_interval': 2.0,
            }],
            output='screen',
        ),
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker',
            namespace='/test_obs',
            output='screen',
        ),
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='listener',
            namespace='/test_obs',
            output='screen',
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestDaemonObservation(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_daemon_obs')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _wait_for_snapshots(self, predicate, timeout=15.0):
        """Wait for a GraphSnapshot that satisfies predicate."""
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        received = []
        sub = self.node.create_subscription(
            GraphSnapshot,
            '/muto/graph_state',
            lambda msg: received.append(msg),
            qos,
        )

        result = None
        start = time.time()
        while result is None and (time.time() - start) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.5)
            for snap in received:
                if predicate(snap):
                    result = snap
                    break

        self.node.destroy_subscription(sub)
        return result

    def test_graph_snapshot_discovers_nodes(self):
        """Verify the daemon publishes graph snapshots with actual nodes."""
        def has_test_nodes(snap):
            fqns = [n.fully_qualified_name for n in snap.actual_nodes]
            return '/test_obs/talker' in fqns and '/test_obs/listener' in fqns

        snapshot = self._wait_for_snapshots(has_test_nodes, timeout=15.0)
        self.assertIsNotNone(
            snapshot, "No snapshot with test nodes received within 15s")

        actual_fqns = [n.fully_qualified_name for n in snapshot.actual_nodes]
        self.assertIn('/test_obs/talker', actual_fqns)
        self.assertIn('/test_obs/listener', actual_fqns)

    def test_convergence_with_desired_state(self):
        """Publish desired state and verify convergence."""
        pub_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        pub = self.node.create_publisher(
            DesiredState, '/muto/daemon/desired_state', pub_qos)

        desired = DesiredState()
        desired.stack_name = 'test-obs-stack'
        # Ignore the test node itself and any launch framework nodes
        desired.ignored_prefixes = ['/test_daemon', '/launch_ros']
        for fqn, name in [
            ('/test_obs/talker', 'talker'),
            ('/test_obs/listener', 'listener'),
        ]:
            ns = NodeState()
            ns.fully_qualified_name = fqn
            ns.name = name
            ns.node_namespace = '/test_obs'
            desired.desired_nodes.append(ns)

        # Subscribe to snapshots
        snap_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        received = []
        sub = self.node.create_subscription(
            GraphSnapshot,
            '/muto/graph_state',
            lambda msg: received.append(msg),
            snap_qos,
        )

        # Repeatedly publish desired state to handle DDS discovery timing
        start = time.time()
        converged = False
        while not converged and (time.time() - start) < 25.0:
            pub.publish(desired)
            rclpy.spin_once(self.node, timeout_sec=0.5)
            for snap in received:
                if snap.status == 'converged':
                    converged = True
                    break

        self.node.destroy_subscription(sub)
        self.node.destroy_publisher(pub)

        self.assertTrue(converged, "Did not reach converged state within 25s")


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)

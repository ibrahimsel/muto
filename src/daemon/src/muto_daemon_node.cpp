// Copyright (c) 2025 Composiv.ai
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0.
//
// SPDX-License-Identifier: EPL-2.0

#include "muto_daemon/muto_daemon_node.hpp"

#include <chrono>
#include <map>

namespace muto_daemon
{

MutoDaemonNode::MutoDaemonNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("muto_daemon", options)
{
  this->declare_parameter("graph_observer_interval", 5.0);
  this->declare_parameter("ignored_prefixes", std::vector<std::string>{});

  graph_observer_interval_ =
    this->get_parameter("graph_observer_interval").as_double();
  ignored_prefixes_ =
    this->get_parameter("ignored_prefixes").as_string_array();

  prober_ = std::make_unique<ActualStateProber>(this);
  resolver_ = std::make_unique<DesiredStateResolver>(this);
  detector_ = std::make_unique<DriftDetector>();
  publisher_ = std::make_unique<GraphStatePublisher>(this);

  prober_->set_system_prefixes({
    "/_",
    "/rosout",
    "/muto_daemon",
  });

  auto period = std::chrono::duration<double>(graph_observer_interval_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&MutoDaemonNode::on_timer, this));

  RCLCPP_INFO(
    this->get_logger(),
    "Muto Daemon started (interval=%.1fs, ignored_prefixes=%zu)",
    graph_observer_interval_, ignored_prefixes_.size());
}

void MutoDaemonNode::on_timer()
{
  auto actual_nodes = prober_->probe();
  auto desired_opt = resolver_->get_desired_state();
  bool paused = resolver_->is_paused();

  muto_msgs::msg::GraphSnapshot snapshot;
  snapshot.timestamp = this->now();
  snapshot.actual_nodes = actual_nodes;

  if (desired_opt.has_value()) {
    const auto & desired = desired_opt.value();
    snapshot.stack_name = desired.stack_name;
    snapshot.stack_id = desired.stack_id;
    snapshot.stack_version = desired.stack_version;
    snapshot.desired_nodes = desired.desired_nodes;

    // Combine node-level and desired-state ignored prefixes
    auto all_ignored = ignored_prefixes_;
    all_ignored.insert(
      all_ignored.end(),
      desired.ignored_prefixes.begin(),
      desired.ignored_prefixes.end());

    auto drift_result = detector_->detect(
      desired.desired_nodes, actual_nodes, all_ignored, paused);

    snapshot.drift = drift_result.drift;
    snapshot.actual_nodes = drift_result.annotated_actual;

    if (paused) {
      snapshot.status = "reconciling";
    } else if (drift_result.drift.missing_nodes.empty() &&
      drift_result.drift.unexpected_nodes.empty())
    {
      snapshot.status = "converged";
    } else {
      snapshot.status = "drifted";
    }

    if (!drift_result.drift.missing_nodes.empty() ||
      !drift_result.drift.unexpected_nodes.empty())
    {
      snapshot.drift.timestamp = this->now();
      publisher_->publish_drift(snapshot.drift);
    }
  } else {
    snapshot.status = "unknown";
  }

  if (desired_opt.has_value()) {
    emit_graph_events(actual_nodes, desired_opt->stack_name);
  }

  publisher_->publish_snapshot(snapshot);

  previous_actual_fqns_.clear();
  for (const auto & node : actual_nodes) {
    previous_actual_fqns_.insert(node.fully_qualified_name);
  }
}

void MutoDaemonNode::emit_graph_events(
  const std::vector<muto_msgs::msg::NodeState> & current_actual,
  const std::string & stack_name)
{
  std::set<std::string> current_fqns;
  std::map<std::string, const muto_msgs::msg::NodeState *> current_map;
  for (const auto & node : current_actual) {
    current_fqns.insert(node.fully_qualified_name);
    current_map[node.fully_qualified_name] = &node;
  }

  // Nodes that appeared
  for (const auto & fqn : current_fqns) {
    if (previous_actual_fqns_.find(fqn) == previous_actual_fqns_.end()) {
      muto_msgs::msg::GraphEvent event;
      event.timestamp = this->now();
      event.event_type = "node_appeared";
      event.stack_name = stack_name;
      auto it = current_map.find(fqn);
      if (it != current_map.end()) {
        event.node_name = it->second->name;
        event.node_namespace = it->second->node_namespace;
      }
      event.details = "Node appeared: " + fqn;
      publisher_->publish_event(event);
    }
  }

  // Nodes that disappeared
  for (const auto & fqn : previous_actual_fqns_) {
    if (current_fqns.find(fqn) == current_fqns.end()) {
      muto_msgs::msg::GraphEvent event;
      event.timestamp = this->now();
      event.event_type = "node_disappeared";
      event.stack_name = stack_name;
      auto last_slash = fqn.rfind('/');
      if (last_slash != std::string::npos) {
        event.node_name = fqn.substr(last_slash + 1);
        event.node_namespace = fqn.substr(0, last_slash);
        if (event.node_namespace.empty()) {
          event.node_namespace = "/";
        }
      }
      event.details = "Node disappeared: " + fqn;
      publisher_->publish_event(event);
    }
  }
}

}  // namespace muto_daemon

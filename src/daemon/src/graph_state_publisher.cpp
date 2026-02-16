// Copyright (c) 2025 Composiv.ai
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0.
//
// SPDX-License-Identifier: EPL-2.0

#include "muto_daemon/graph_state_publisher.hpp"

#include <nlohmann/json.hpp>

namespace muto_daemon
{

GraphStatePublisher::GraphStatePublisher(rclcpp::Node * node)
: node_(node)
{
  auto state_qos = rclcpp::QoS(1).reliable().transient_local();
  auto drift_qos = rclcpp::QoS(1).reliable().transient_local();
  auto event_qos = rclcpp::QoS(50).reliable();
  auto json_qos = rclcpp::QoS(1).best_effort();

  snapshot_pub_ = node_->create_publisher<muto_msgs::msg::GraphSnapshot>(
    "/muto/graph_state", state_qos);
  drift_pub_ = node_->create_publisher<muto_msgs::msg::GraphDrift>(
    "/muto/graph_drift", drift_qos);
  event_pub_ = node_->create_publisher<muto_msgs::msg::GraphEvent>(
    "/muto/graph_events", event_qos);
  json_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/muto/graph_state_json", json_qos);

  get_state_srv_ = node_->create_service<muto_msgs::srv::GetGraphState>(
    "/muto/get_graph_state",
    std::bind(
      &GraphStatePublisher::on_get_graph_state, this,
      std::placeholders::_1, std::placeholders::_2));
}

void GraphStatePublisher::publish_snapshot(
  const muto_msgs::msg::GraphSnapshot & snapshot)
{
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    latest_snapshot_ = snapshot;
  }

  snapshot_pub_->publish(snapshot);

  auto json_msg = std_msgs::msg::String();
  json_msg.data = snapshot_to_json(snapshot);
  json_pub_->publish(json_msg);
}

void GraphStatePublisher::publish_drift(
  const muto_msgs::msg::GraphDrift & drift)
{
  drift_pub_->publish(drift);
}

void GraphStatePublisher::publish_event(
  const muto_msgs::msg::GraphEvent & event)
{
  event_pub_->publish(event);
}

void GraphStatePublisher::on_get_graph_state(
  const muto_msgs::srv::GetGraphState::Request::SharedPtr /*request*/,
  muto_msgs::srv::GetGraphState::Response::SharedPtr response)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  response->snapshot = latest_snapshot_;
  response->success = true;
}

std::string GraphStatePublisher::snapshot_to_json(
  const muto_msgs::msg::GraphSnapshot & snapshot) const
{
  nlohmann::json j;
  j["timestamp"]["sec"] = snapshot.timestamp.sec;
  j["timestamp"]["nanosec"] = snapshot.timestamp.nanosec;
  j["stack_name"] = snapshot.stack_name;
  j["stack_id"] = snapshot.stack_id;
  j["stack_version"] = snapshot.stack_version;
  j["status"] = snapshot.status;

  auto node_to_json = [](const muto_msgs::msg::NodeState & n) {
      nlohmann::json nj;
      nj["name"] = n.name;
      nj["node_namespace"] = n.node_namespace;
      nj["fqn"] = n.fully_qualified_name;
      nj["status"] = n.status;
      nj["managed_by_muto"] = n.managed_by_muto;
      nj["publisher_topics"] = n.publisher_topics;
      nj["subscriber_topics"] = n.subscriber_topics;
      nj["service_names"] = n.service_names;
      return nj;
    };

  j["desired_nodes"] = nlohmann::json::array();
  for (const auto & n : snapshot.desired_nodes) {
    j["desired_nodes"].push_back(node_to_json(n));
  }

  j["actual_nodes"] = nlohmann::json::array();
  for (const auto & n : snapshot.actual_nodes) {
    j["actual_nodes"].push_back(node_to_json(n));
  }

  j["drift"]["missing_nodes"] = snapshot.drift.missing_nodes;
  j["drift"]["unexpected_nodes"] = snapshot.drift.unexpected_nodes;

  return j.dump();
}

}  // namespace muto_daemon

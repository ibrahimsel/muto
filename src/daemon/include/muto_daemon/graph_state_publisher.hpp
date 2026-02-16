// Copyright (c) 2025 Composiv.ai
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0.
//
// SPDX-License-Identifier: EPL-2.0

#ifndef MUTO_DAEMON__GRAPH_STATE_PUBLISHER_HPP_
#define MUTO_DAEMON__GRAPH_STATE_PUBLISHER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "muto_msgs/msg/graph_drift.hpp"
#include "muto_msgs/msg/graph_event.hpp"
#include "muto_msgs/msg/graph_snapshot.hpp"
#include "muto_msgs/srv/get_graph_state.hpp"
#include "std_msgs/msg/string.hpp"

namespace muto_daemon
{

class GraphStatePublisher
{
public:
  explicit GraphStatePublisher(rclcpp::Node * node);

  void publish_snapshot(const muto_msgs::msg::GraphSnapshot & snapshot);
  void publish_drift(const muto_msgs::msg::GraphDrift & drift);
  void publish_event(const muto_msgs::msg::GraphEvent & event);

private:
  void on_get_graph_state(
    const muto_msgs::srv::GetGraphState::Request::SharedPtr request,
    muto_msgs::srv::GetGraphState::Response::SharedPtr response);

  std::string snapshot_to_json(
    const muto_msgs::msg::GraphSnapshot & snapshot) const;

  rclcpp::Node * node_;
  rclcpp::Publisher<muto_msgs::msg::GraphSnapshot>::SharedPtr snapshot_pub_;
  rclcpp::Publisher<muto_msgs::msg::GraphDrift>::SharedPtr drift_pub_;
  rclcpp::Publisher<muto_msgs::msg::GraphEvent>::SharedPtr event_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr json_pub_;
  rclcpp::Service<muto_msgs::srv::GetGraphState>::SharedPtr get_state_srv_;

  muto_msgs::msg::GraphSnapshot latest_snapshot_;
  std::mutex snapshot_mutex_;
};

}  // namespace muto_daemon

#endif  // MUTO_DAEMON__GRAPH_STATE_PUBLISHER_HPP_

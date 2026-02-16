// Copyright (c) 2025 Composiv.ai
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0.
//
// SPDX-License-Identifier: EPL-2.0

#ifndef MUTO_DAEMON__MUTO_DAEMON_NODE_HPP_
#define MUTO_DAEMON__MUTO_DAEMON_NODE_HPP_

#include <memory>
#include <set>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "muto_daemon/actual_state_prober.hpp"
#include "muto_daemon/desired_state_resolver.hpp"
#include "muto_daemon/drift_detector.hpp"
#include "muto_daemon/graph_state_publisher.hpp"

namespace muto_daemon
{

class MutoDaemonNode : public rclcpp::Node
{
public:
  explicit MutoDaemonNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_timer();

  void emit_graph_events(
    const std::vector<muto_msgs::msg::NodeState> & current_actual,
    const std::string & stack_name);

  std::unique_ptr<ActualStateProber> prober_;
  std::unique_ptr<DesiredStateResolver> resolver_;
  std::unique_ptr<DriftDetector> detector_;
  std::unique_ptr<GraphStatePublisher> publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::set<std::string> previous_actual_fqns_;

  double graph_observer_interval_;
  std::vector<std::string> ignored_prefixes_;
};

}  // namespace muto_daemon

#endif  // MUTO_DAEMON__MUTO_DAEMON_NODE_HPP_

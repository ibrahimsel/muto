// Copyright (c) 2025 Composiv.ai
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0.
//
// SPDX-License-Identifier: EPL-2.0

#ifndef MUTO_DAEMON__ACTUAL_STATE_PROBER_HPP_
#define MUTO_DAEMON__ACTUAL_STATE_PROBER_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "muto_msgs/msg/node_state.hpp"

namespace muto_daemon
{

class ActualStateProber
{
public:
  explicit ActualStateProber(rclcpp::Node * node);

  std::vector<muto_msgs::msg::NodeState> probe();

  void set_system_prefixes(const std::vector<std::string> & prefixes);

private:
  bool is_system_node(const std::string & name, const std::string & ns) const;
  std::string make_fqn(const std::string & name, const std::string & ns) const;

  rclcpp::Node * node_;
  std::vector<std::string> system_prefixes_;
};

}  // namespace muto_daemon

#endif  // MUTO_DAEMON__ACTUAL_STATE_PROBER_HPP_

// Copyright (c) 2025 Composiv.ai
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0.
//
// SPDX-License-Identifier: EPL-2.0

#ifndef MUTO_DAEMON__DESIRED_STATE_RESOLVER_HPP_
#define MUTO_DAEMON__DESIRED_STATE_RESOLVER_HPP_

#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "muto_msgs/msg/desired_state.hpp"

namespace muto_daemon
{

class DesiredStateResolver
{
public:
  explicit DesiredStateResolver(
    rclcpp::Node * node, const std::string & cache_dir = "");

  std::optional<muto_msgs::msg::DesiredState> get_desired_state() const;
  bool is_paused() const;
  bool has_desired_state() const;

private:
  void on_desired_state(const muto_msgs::msg::DesiredState::SharedPtr msg);
  void write_cache(const muto_msgs::msg::DesiredState & state);
  bool load_cache();
  std::string get_cache_path() const;

  rclcpp::Node * node_;
  rclcpp::Subscription<muto_msgs::msg::DesiredState>::SharedPtr sub_;
  mutable std::mutex mutex_;
  std::optional<muto_msgs::msg::DesiredState> desired_state_;
  std::string cache_dir_;
};

}  // namespace muto_daemon

#endif  // MUTO_DAEMON__DESIRED_STATE_RESOLVER_HPP_

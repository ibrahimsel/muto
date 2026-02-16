// Copyright (c) 2025 Composiv.ai
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0.
//
// SPDX-License-Identifier: EPL-2.0

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "muto_daemon/muto_daemon_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<muto_daemon::MutoDaemonNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

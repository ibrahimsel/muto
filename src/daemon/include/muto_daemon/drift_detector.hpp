// Copyright (c) 2025 Composiv.ai
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0.
//
// SPDX-License-Identifier: EPL-2.0

#ifndef MUTO_DAEMON__DRIFT_DETECTOR_HPP_
#define MUTO_DAEMON__DRIFT_DETECTOR_HPP_

#include <string>
#include <vector>

#include "muto_msgs/msg/graph_drift.hpp"
#include "muto_msgs/msg/node_state.hpp"

namespace muto_daemon
{

struct DriftResult
{
  muto_msgs::msg::GraphDrift drift;
  std::vector<muto_msgs::msg::NodeState> annotated_actual;
};

class DriftDetector
{
public:
  DriftResult detect(
    const std::vector<muto_msgs::msg::NodeState> & desired,
    const std::vector<muto_msgs::msg::NodeState> & actual,
    const std::vector<std::string> & ignored_prefixes,
    bool paused = false);

private:
  bool matches_prefix(
    const std::string & fqn,
    const std::vector<std::string> & prefixes) const;
};

}  // namespace muto_daemon

#endif  // MUTO_DAEMON__DRIFT_DETECTOR_HPP_

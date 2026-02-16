// Copyright (c) 2025 Composiv.ai
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0.
//
// SPDX-License-Identifier: EPL-2.0

#include "muto_daemon/drift_detector.hpp"

#include <set>

namespace muto_daemon
{

bool DriftDetector::matches_prefix(
  const std::string & fqn,
  const std::vector<std::string> & prefixes) const
{
  for (const auto & prefix : prefixes) {
    if (fqn.compare(0, prefix.size(), prefix) == 0) {
      return true;
    }
  }
  return false;
}

DriftResult DriftDetector::detect(
  const std::vector<muto_msgs::msg::NodeState> & desired,
  const std::vector<muto_msgs::msg::NodeState> & actual,
  const std::vector<std::string> & ignored_prefixes,
  bool paused)
{
  DriftResult result;
  result.annotated_actual = actual;

  // No drift when paused or no desired state to compare against
  if (paused || desired.empty()) {
    return result;
  }

  // Build desired FQN set
  std::set<std::string> desired_fqns;
  for (const auto & node : desired) {
    desired_fqns.insert(node.fully_qualified_name);
  }

  // Build actual FQN set
  std::set<std::string> actual_fqns;
  for (const auto & node : actual) {
    actual_fqns.insert(node.fully_qualified_name);
  }

  // Missing: in desired but not in actual
  for (const auto & fqn : desired_fqns) {
    if (actual_fqns.find(fqn) == actual_fqns.end()) {
      result.drift.missing_nodes.push_back(fqn);
    }
  }

  // Unexpected: in actual but not in desired, excluding ignored prefixes
  for (const auto & fqn : actual_fqns) {
    if (desired_fqns.find(fqn) == desired_fqns.end()) {
      if (!matches_prefix(fqn, ignored_prefixes)) {
        result.drift.unexpected_nodes.push_back(fqn);
      }
    }
  }

  // Annotate actual nodes with managed_by_muto flag
  for (auto & node : result.annotated_actual) {
    node.managed_by_muto =
      desired_fqns.find(node.fully_qualified_name) != desired_fqns.end();
  }

  return result;
}

}  // namespace muto_daemon

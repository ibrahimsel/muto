// Copyright (c) 2025 Composiv.ai
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0.
//
// SPDX-License-Identifier: EPL-2.0

#include "muto_daemon/actual_state_prober.hpp"

#include <map>

namespace muto_daemon
{

ActualStateProber::ActualStateProber(rclcpp::Node * node)
: node_(node)
{
  system_prefixes_ = {
    "/_",
    "/rosout",
    "/muto_daemon",
  };
}

void ActualStateProber::set_system_prefixes(
  const std::vector<std::string> & prefixes)
{
  system_prefixes_ = prefixes;
}

std::string ActualStateProber::make_fqn(
  const std::string & name, const std::string & ns) const
{
  if (ns.empty() || ns == "/") {
    return "/" + name;
  }
  std::string clean_ns = ns;
  if (clean_ns.back() == '/') {
    clean_ns.pop_back();
  }
  return clean_ns + "/" + name;
}

bool ActualStateProber::is_system_node(
  const std::string & name, const std::string & ns) const
{
  std::string fqn = make_fqn(name, ns);
  for (const auto & prefix : system_prefixes_) {
    if (fqn.compare(0, prefix.size(), prefix) == 0) {
      return true;
    }
  }
  return false;
}

std::vector<muto_msgs::msg::NodeState> ActualStateProber::probe()
{
  std::vector<muto_msgs::msg::NodeState> result;
  auto node_names_ns =
    node_->get_node_graph_interface()->get_node_names_and_namespaces();

  // Build topic maps for enrichment (single pass over all topics)
  std::map<std::string, std::vector<std::string>> pub_topics;
  std::map<std::string, std::vector<std::string>> sub_topics;

  try {
    auto all_topics = node_->get_topic_names_and_types();
    for (const auto & [topic_name, types] : all_topics) {
      try {
        for (const auto & ep : node_->get_publishers_info_by_topic(topic_name)) {
          std::string fqn = make_fqn(ep.node_name(), ep.node_namespace());
          pub_topics[fqn].push_back(topic_name);
        }
        for (const auto & ep : node_->get_subscriptions_info_by_topic(topic_name)) {
          std::string fqn = make_fqn(ep.node_name(), ep.node_namespace());
          sub_topics[fqn].push_back(topic_name);
        }
      } catch (...) {
        // Skip topics that fail to query
      }
    }
  } catch (...) {
    RCLCPP_WARN(node_->get_logger(), "Failed to enumerate topics for enrichment");
  }

  for (const auto & [name, ns] : node_names_ns) {
    if (is_system_node(name, ns)) {
      continue;
    }

    muto_msgs::msg::NodeState state;
    state.name = name;
    state.node_namespace = ns;
    state.fully_qualified_name = make_fqn(name, ns);
    state.status = "running";
    state.managed_by_muto = false;
    // package_name and executable are empty (not available from graph introspection)

    auto pub_it = pub_topics.find(state.fully_qualified_name);
    if (pub_it != pub_topics.end()) {
      state.publisher_topics = pub_it->second;
    }
    auto sub_it = sub_topics.find(state.fully_qualified_name);
    if (sub_it != sub_topics.end()) {
      state.subscriber_topics = sub_it->second;
    }

    try {
      auto services = node_->get_service_names_and_types_by_node(name, ns);
      for (const auto & [srv_name, srv_types] : services) {
        state.service_names.push_back(srv_name);
      }
    } catch (...) {
      // Service query may fail for some nodes
    }

    result.push_back(std::move(state));
  }

  return result;
}

}  // namespace muto_daemon

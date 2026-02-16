// Copyright (c) 2025 Composiv.ai
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0.
//
// SPDX-License-Identifier: EPL-2.0

#include "muto_daemon/desired_state_resolver.hpp"

#include <filesystem>
#include <fstream>

#include <nlohmann/json.hpp>

namespace muto_daemon
{

DesiredStateResolver::DesiredStateResolver(
  rclcpp::Node * node, const std::string & cache_dir)
: node_(node), cache_dir_(cache_dir)
{
  if (cache_dir_.empty()) {
    const char * home = std::getenv("HOME");
    if (home) {
      cache_dir_ = std::string(home) + "/.muto/daemon";
    } else {
      cache_dir_ = "/tmp/muto/daemon";
    }
  }

  auto qos = rclcpp::QoS(1).reliable().transient_local();
  sub_ = node_->create_subscription<muto_msgs::msg::DesiredState>(
    "/muto/daemon/desired_state", qos,
    std::bind(
      &DesiredStateResolver::on_desired_state, this, std::placeholders::_1));

  if (load_cache()) {
    RCLCPP_INFO(node_->get_logger(), "Loaded desired state from cache");
  }
}

std::optional<muto_msgs::msg::DesiredState>
DesiredStateResolver::get_desired_state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return desired_state_;
}

bool DesiredStateResolver::is_paused() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return desired_state_.has_value() && desired_state_->paused;
}

bool DesiredStateResolver::has_desired_state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return desired_state_.has_value();
}

void DesiredStateResolver::on_desired_state(
  const muto_msgs::msg::DesiredState::SharedPtr msg)
{
  RCLCPP_INFO(
    node_->get_logger(),
    "Received desired state for stack: %s (paused=%s)",
    msg->stack_name.c_str(), msg->paused ? "true" : "false");

  {
    std::lock_guard<std::mutex> lock(mutex_);
    desired_state_ = *msg;
  }

  write_cache(*msg);
}

std::string DesiredStateResolver::get_cache_path() const
{
  return cache_dir_ + "/desired_state_cache.json";
}

void DesiredStateResolver::write_cache(
  const muto_msgs::msg::DesiredState & state)
{
  try {
    std::filesystem::create_directories(cache_dir_);

    nlohmann::json j;
    j["stack_name"] = state.stack_name;
    j["stack_id"] = state.stack_id;
    j["stack_version"] = state.stack_version;
    j["paused"] = state.paused;
    j["ignored_prefixes"] = state.ignored_prefixes;

    nlohmann::json nodes = nlohmann::json::array();
    for (const auto & node : state.desired_nodes) {
      nlohmann::json n;
      n["name"] = node.name;
      n["node_namespace"] = node.node_namespace;
      n["fully_qualified_name"] = node.fully_qualified_name;
      n["package_name"] = node.package_name;
      n["executable"] = node.executable;
      nodes.push_back(n);
    }
    j["desired_nodes"] = nodes;

    std::ofstream ofs(get_cache_path());
    ofs << j.dump(2);

    RCLCPP_DEBUG(
      node_->get_logger(), "Wrote desired state cache to %s",
      get_cache_path().c_str());
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      node_->get_logger(), "Failed to write desired state cache: %s",
      e.what());
  }
}

bool DesiredStateResolver::load_cache()
{
  try {
    std::ifstream ifs(get_cache_path());
    if (!ifs.is_open()) {
      return false;
    }

    nlohmann::json j;
    ifs >> j;

    muto_msgs::msg::DesiredState state;
    state.stack_name = j.value("stack_name", "");
    state.stack_id = j.value("stack_id", "");
    state.stack_version = j.value("stack_version", "");
    state.paused = j.value("paused", false);

    if (j.contains("ignored_prefixes")) {
      for (const auto & p : j["ignored_prefixes"]) {
        state.ignored_prefixes.push_back(p.get<std::string>());
      }
    }

    if (j.contains("desired_nodes")) {
      for (const auto & n : j["desired_nodes"]) {
        muto_msgs::msg::NodeState node;
        node.name = n.value("name", "");
        node.node_namespace = n.value("node_namespace", "");
        node.fully_qualified_name = n.value("fully_qualified_name", "");
        node.package_name = n.value("package_name", "");
        node.executable = n.value("executable", "");
        state.desired_nodes.push_back(node);
      }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    desired_state_ = state;
    return true;
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      node_->get_logger(), "Failed to load desired state cache: %s",
      e.what());
    return false;
  }
}

}  // namespace muto_daemon

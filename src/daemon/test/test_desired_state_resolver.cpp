// Copyright (c) 2025 Composiv.ai
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0.
//
// SPDX-License-Identifier: EPL-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "muto_daemon/desired_state_resolver.hpp"

using muto_msgs::msg::DesiredState;
using muto_msgs::msg::NodeState;

class DesiredStateResolverTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    cache_dir_ = "/tmp/muto_test_" + std::to_string(getpid()) + "_" +
      std::to_string(test_count_++);
    std::filesystem::create_directories(cache_dir_);
    node_ = std::make_shared<rclcpp::Node>(
      "test_resolver_" + std::to_string(test_count_));
  }

  void TearDown() override
  {
    node_.reset();
    std::filesystem::remove_all(cache_dir_);
  }

  rclcpp::Node::SharedPtr node_;
  std::string cache_dir_;
  static int test_count_;
};

int DesiredStateResolverTest::test_count_ = 0;

TEST_F(DesiredStateResolverTest, CachesReceivedState)
{
  auto resolver = std::make_unique<muto_daemon::DesiredStateResolver>(
    node_.get(), cache_dir_);

  auto pub = node_->create_publisher<DesiredState>(
    "/muto/daemon/desired_state",
    rclcpp::QoS(1).reliable().transient_local());

  DesiredState msg;
  msg.stack_name = "test-stack";
  msg.stack_id = "org.test:stack";
  msg.paused = false;

  NodeState ns;
  ns.fully_qualified_name = "/test/node_a";
  ns.name = "node_a";
  ns.node_namespace = "/test";
  msg.desired_nodes.push_back(ns);

  pub->publish(msg);

  auto start = std::chrono::steady_clock::now();
  while (!resolver->has_desired_state() &&
    (std::chrono::steady_clock::now() - start) < std::chrono::seconds(2))
  {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_TRUE(resolver->has_desired_state());
  auto state = resolver->get_desired_state();
  EXPECT_EQ(state->stack_name, "test-stack");
  EXPECT_EQ(state->desired_nodes.size(), 1u);
}

TEST_F(DesiredStateResolverTest, FallbackToFile)
{
  std::filesystem::create_directories(cache_dir_);
  std::ofstream ofs(cache_dir_ + "/desired_state_cache.json");
  ofs << R"({
    "stack_name": "cached-stack",
    "stack_id": "org.test:cached",
    "stack_version": "1.0.0",
    "paused": false,
    "ignored_prefixes": ["/rviz"],
    "desired_nodes": [
      {
        "name": "cached_node",
        "node_namespace": "/cached",
        "fully_qualified_name": "/cached/cached_node",
        "package_name": "",
        "executable": ""
      }
    ]
  })";
  ofs.close();

  auto resolver = std::make_unique<muto_daemon::DesiredStateResolver>(
    node_.get(), cache_dir_);

  ASSERT_TRUE(resolver->has_desired_state());
  auto state = resolver->get_desired_state();
  EXPECT_EQ(state->stack_name, "cached-stack");
  EXPECT_EQ(state->desired_nodes.size(), 1u);
  EXPECT_EQ(
    state->desired_nodes[0].fully_qualified_name, "/cached/cached_node");
  EXPECT_EQ(state->ignored_prefixes.size(), 1u);
  EXPECT_EQ(state->ignored_prefixes[0], "/rviz");
}

TEST_F(DesiredStateResolverTest, EmptyOnColdStart)
{
  auto resolver = std::make_unique<muto_daemon::DesiredStateResolver>(
    node_.get(), cache_dir_);

  EXPECT_FALSE(resolver->has_desired_state());
  EXPECT_FALSE(resolver->is_paused());
}

TEST_F(DesiredStateResolverTest, WritesCache)
{
  auto resolver = std::make_unique<muto_daemon::DesiredStateResolver>(
    node_.get(), cache_dir_);

  auto pub = node_->create_publisher<DesiredState>(
    "/muto/daemon/desired_state",
    rclcpp::QoS(1).reliable().transient_local());

  DesiredState msg;
  msg.stack_name = "write-test";
  pub->publish(msg);

  auto start = std::chrono::steady_clock::now();
  while (!resolver->has_desired_state() &&
    (std::chrono::steady_clock::now() - start) < std::chrono::seconds(2))
  {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_TRUE(
    std::filesystem::exists(cache_dir_ + "/desired_state_cache.json"));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

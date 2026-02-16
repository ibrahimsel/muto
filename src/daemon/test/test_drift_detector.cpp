// Copyright (c) 2025 Composiv.ai
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0.
//
// SPDX-License-Identifier: EPL-2.0

#include <gtest/gtest.h>

#include "muto_daemon/drift_detector.hpp"

using muto_msgs::msg::NodeState;
using muto_daemon::DriftDetector;

static NodeState make_node(const std::string & fqn)
{
  NodeState n;
  n.fully_qualified_name = fqn;
  auto last_slash = fqn.rfind('/');
  if (last_slash != std::string::npos && last_slash > 0) {
    n.name = fqn.substr(last_slash + 1);
    n.node_namespace = fqn.substr(0, last_slash);
  } else if (fqn.size() > 1) {
    n.name = fqn.substr(1);
    n.node_namespace = "/";
  }
  n.status = "running";
  return n;
}

TEST(DriftDetector, NoDesiredState_NoDrift)
{
  DriftDetector detector;
  std::vector<NodeState> desired;
  std::vector<NodeState> actual = {make_node("/some_node")};

  auto result = detector.detect(desired, actual, {});

  EXPECT_TRUE(result.drift.missing_nodes.empty());
  EXPECT_TRUE(result.drift.unexpected_nodes.empty());
}

TEST(DriftDetector, AllPresent_Converged)
{
  DriftDetector detector;
  std::vector<NodeState> desired = {make_node("/A"), make_node("/B")};
  std::vector<NodeState> actual = {make_node("/A"), make_node("/B")};

  auto result = detector.detect(desired, actual, {});

  EXPECT_TRUE(result.drift.missing_nodes.empty());
  EXPECT_TRUE(result.drift.unexpected_nodes.empty());
}

TEST(DriftDetector, MissingNode_Detected)
{
  DriftDetector detector;
  std::vector<NodeState> desired = {make_node("/A"), make_node("/B")};
  std::vector<NodeState> actual = {make_node("/A")};

  auto result = detector.detect(desired, actual, {});

  ASSERT_EQ(result.drift.missing_nodes.size(), 1u);
  EXPECT_EQ(result.drift.missing_nodes[0], "/B");
  EXPECT_TRUE(result.drift.unexpected_nodes.empty());
}

TEST(DriftDetector, UnexpectedNode_Detected)
{
  DriftDetector detector;
  std::vector<NodeState> desired = {make_node("/A")};
  std::vector<NodeState> actual = {make_node("/A"), make_node("/C")};

  auto result = detector.detect(desired, actual, {});

  EXPECT_TRUE(result.drift.missing_nodes.empty());
  ASSERT_EQ(result.drift.unexpected_nodes.size(), 1u);
  EXPECT_EQ(result.drift.unexpected_nodes[0], "/C");
}

TEST(DriftDetector, UnexpectedNode_IgnoredPrefix)
{
  DriftDetector detector;
  std::vector<NodeState> desired = {make_node("/A")};
  std::vector<NodeState> actual = {make_node("/A"), make_node("/rviz/window")};

  auto result = detector.detect(desired, actual, {"/rviz"});

  EXPECT_TRUE(result.drift.missing_nodes.empty());
  EXPECT_TRUE(result.drift.unexpected_nodes.empty());
}

TEST(DriftDetector, SystemNodesFiltered)
{
  DriftDetector detector;
  std::vector<NodeState> desired = {make_node("/A")};
  std::vector<NodeState> actual = {
    make_node("/A"),
    make_node("/rosout"),
    make_node("/_daemon"),
  };

  auto result = detector.detect(desired, actual, {"/rosout", "/_"});

  EXPECT_TRUE(result.drift.missing_nodes.empty());
  EXPECT_TRUE(result.drift.unexpected_nodes.empty());
}

TEST(DriftDetector, Paused_NoDrift)
{
  DriftDetector detector;
  std::vector<NodeState> desired = {make_node("/A"), make_node("/B")};
  std::vector<NodeState> actual = {make_node("/A")};

  auto result = detector.detect(desired, actual, {}, true);

  EXPECT_TRUE(result.drift.missing_nodes.empty());
  EXPECT_TRUE(result.drift.unexpected_nodes.empty());
}

TEST(DriftDetector, ManagedByMuto_Populated)
{
  DriftDetector detector;
  std::vector<NodeState> desired = {make_node("/A")};
  std::vector<NodeState> actual = {make_node("/A"), make_node("/C")};

  auto result = detector.detect(desired, actual, {});

  ASSERT_EQ(result.annotated_actual.size(), 2u);
  bool found_a = false, found_c = false;
  for (const auto & n : result.annotated_actual) {
    if (n.fully_qualified_name == "/A") {
      EXPECT_TRUE(n.managed_by_muto);
      found_a = true;
    }
    if (n.fully_qualified_name == "/C") {
      EXPECT_FALSE(n.managed_by_muto);
      found_c = true;
    }
  }
  EXPECT_TRUE(found_a);
  EXPECT_TRUE(found_c);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

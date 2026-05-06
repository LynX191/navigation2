// Copyright (c) 2022 Neobotix GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__PATH_LONGER_ON_APPROACH_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__PATH_LONGER_ON_APPROACH_HPP_

#include <string>
#include <memory>
#include <limits>
#include <cmath>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

class PathLongerOnApproach : public BT::DecoratorNode
{
public:
  PathLongerOnApproach(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Planned Path"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "goal", "Navigation goal (used for goal-change detection)"),
      BT::InputPort<double>(
        "prox_len", 20.0,
        "Proximity length (m) within which path length growth is monitored"),
      BT::InputPort<double>(
        "length_factor", 1.1,
        "Relative factor: effective_total must exceed length_factor * snapshot_total"),
      BT::InputPort<double>(
        "abs_length", 0.5,
        "Absolute floor (m): effective_total - snapshot_total must also exceed this"),
    };
  }

  BT::NodeStatus tick() override;

private:
  bool isRobotInGoalProximity(
    nav_msgs::msg::Path & path,
    double & prox_len);

  nav_msgs::msg::Path current_path_;
  geometry_msgs::msg::PoseStamped current_goal_;
  geometry_msgs::msg::Point last_goal_position_;

  double prox_len_      = std::numeric_limits<double>::max();
  double length_factor_ = std::numeric_limits<double>::max();
  double abs_length_    = 0.0;

  /// Total path length at snapshot time.
  double snapshot_total_length_ = 0.0;

  /// Remaining path length on the previous tick.
  /// Used to compute the incremental step the robot moved each tick.
  double prev_remaining_ = 0.0;

  /// Accumulated distance the robot has moved since snapshot was taken.
  /// Grows each tick by max(0, prev_remaining - current_remaining).
  double distance_moved_ = 0.0;

  double current_remaining = 0.0;
  double effective_total = 0.0;
  double delta = 0.0;
  rclcpp::Node::SharedPtr node_;

  bool first_time_      = true;
  bool snapshot_taken_  = false;
  bool child_triggered_ = false;

  static constexpr double kGoalPositionTolerance = 0.05;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__PATH_LONGER_ON_APPROACH_HPP_
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

#include <string>
#include <memory>
#include <vector>
#include <cmath>
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/decorator/path_longer_on_approach.hpp"

namespace nav2_behavior_tree
{

PathLongerOnApproach::PathLongerOnApproach(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

bool PathLongerOnApproach::isRobotInGoalProximity(
  nav_msgs::msg::Path & path,
  double & prox_len)
{
  return nav2_util::geometry_utils::calculate_path_length(path, 0) < prox_len;
}

inline BT::NodeStatus PathLongerOnApproach::tick()
{
  getInput("path", current_path_);
  getInput("goal", current_goal_);
  getInput("prox_len", prox_len_);
  getInput("length_factor", length_factor_);
  getInput("abs_length", abs_length_);

  // ── Guard: empty path ─────────────────────────────────────────────────────
  if (current_path_.poses.empty()) {
    return BT::NodeStatus::SUCCESS;
  }

  // ── Goal-change detection (blackboard goal, not path.poses.back()) ────────
  const auto & gp = current_goal_.pose.position;
  if(gp.x != last_goal_position_.x || gp.y != last_goal_position_.y) {
    RCLCPP_INFO(node_->get_logger(), "Goal change detected: (%.3f,%.3f)->(%.3f,%.3f)", last_goal_position_.x, last_goal_position_.y, gp.x, gp.y);
  }
  if (!first_time_) {
    const double dx = gp.x - last_goal_position_.x;
    const double dy = gp.y - last_goal_position_.y;
    if (std::sqrt(dx * dx + dy * dy) > kGoalPositionTolerance) {
      RCLCPP_INFO(
        node_->get_logger(),
        "PathLongerOnApproach: goal changed (%.3f,%.3f)->(%.3f,%.3f) — resetting.",
        last_goal_position_.x, last_goal_position_.y,
        gp.x, gp.y);
      snapshot_taken_   = false;
      child_triggered_  = false;
      distance_moved_   = 0.0;
      resetChild();
    }
  }
  last_goal_position_ = gp;
  first_time_         = false;

  setStatus(BT::NodeStatus::RUNNING);

  // ── Proximity check ───────────────────────────────────────────────────────
  if (!isRobotInGoalProximity(current_path_, prox_len_)) {
    snapshot_taken_   = false;
    child_triggered_  = false;
    distance_moved_   = 0.0;
    return BT::NodeStatus::SUCCESS;
  }

  // ── Current remaining length ──────────────────────────────────────────────
  bool is_current_remaining_diff = current_remaining != nav2_util::geometry_utils::calculate_path_length(current_path_, 0);
  current_remaining =
    nav2_util::geometry_utils::calculate_path_length(current_path_, 0);

  // ── Snapshot on first proximity entry ────────────────────────────────────
  if (!snapshot_taken_) {
    snapshot_total_length_ = current_remaining;
    prev_remaining_        = current_remaining;
    distance_moved_        = 0.0;
    snapshot_taken_        = true;
    RCLCPP_INFO(
      node_->get_logger(),
      "PathLongerOnApproach: snapshot taken — total=%.3f m",
      snapshot_total_length_);
    return BT::NodeStatus::SUCCESS;
  }

  // ── Accumulate distance moved ─────────────────────────────────────────────
  // Each tick, if the robot moved forward (remaining decreased), add the
  // difference to the running distance_moved_ counter.
  // Cap at zero to ignore backward movement or path length noise.
  const double step = prev_remaining_ - current_remaining;
  if (step > 0.0) {
    distance_moved_ += step;
  }
  prev_remaining_ = current_remaining;

  // ── Effective total length ────────────────────────────────────────────────
  // effective_total = distance already moved + remaining on current path
  // This reconstructs the "total journey length" as seen right now.
  // If the path got longer, effective_total grows above snapshot_total_length_.
  // If the robot just moved forward on the same path, effective_total stays
  // approximately equal to snapshot_total_length_.

  bool is_effective_total_diff = effective_total != distance_moved_ + current_remaining;
  bool is_delta_diff = delta != effective_total - snapshot_total_length_;
  effective_total = distance_moved_ + current_remaining;
  delta           = effective_total - snapshot_total_length_;



  if(is_current_remaining_diff || is_effective_total_diff || is_delta_diff) {
    RCLCPP_INFO(
      node_->get_logger(),
      "PathLongerOnApproach: snapshot=%.3f m, moved=%.3f m, "
      "remaining=%.3f m, effective=%.3f m, delta=%.3f m "
      "(need delta>%.3f AND ratio>%.3f)",
      snapshot_total_length_, distance_moved_, current_remaining,
      effective_total, delta,
      abs_length_, length_factor_);
  }

  // ── If child already running, keep ticking it ────────────────────────────
  if (child_triggered_) {
    const BT::NodeStatus child_state = child_node_->executeTick();
    switch (child_state) {
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;
      case BT::NodeStatus::SUCCESS:
      case BT::NodeStatus::FAILURE:
        child_triggered_ = false;
        resetChild();
        return child_state;
      default:
        child_triggered_ = false;
        return BT::NodeStatus::FAILURE;
    }
  }

  // ── Main trigger condition ────────────────────────────────────────────────
  // const bool relative_exceeded = effective_total > length_factor_ * snapshot_total_length_;
  const bool absolute_exceeded = delta > abs_length_;

  if (absolute_exceeded) {
    RCLCPP_INFO(
      node_->get_logger(),
      "PathLongerOnApproach: triggered — effective=%.3f m > "
      "%.3f x snapshot=%.3f m, delta=%.3f m > abs=%.3f m",
      effective_total, length_factor_, snapshot_total_length_,
      delta, abs_length_);

    // Advance snapshot to current state so we don't re-trigger immediately.
    snapshot_total_length_ = effective_total;
    prev_remaining_        = current_remaining;
    distance_moved_        = 0.0;
    child_triggered_       = true;

    const BT::NodeStatus child_state = child_node_->executeTick();
    switch (child_state) {
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;
      case BT::NodeStatus::SUCCESS:
      case BT::NodeStatus::FAILURE:
        child_triggered_ = false;
        resetChild();
        return child_state;
      default:
        child_triggered_ = false;
        return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PathLongerOnApproach>("PathLongerOnApproach");
}
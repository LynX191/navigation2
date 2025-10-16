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
#include <chrono>
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/decorator/path_length_checker.hpp"

namespace nav2_behavior_tree
{

PathLengthChecker::PathLengthChecker(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  wait_timeout_(5.0),
  check_frequency_(10.0),
  enable_waiting_(true),
  is_waiting_(false),
  longer_path_detected_(false)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

bool PathLengthChecker::isPathUpdated(
  nav_msgs::msg::Path & new_path,
  nav_msgs::msg::Path & old_path)
{
  return old_path.poses.size() != 0 &&
         new_path.poses.size() != 0 &&
         new_path.poses.size() != old_path.poses.size() &&
         old_path.poses.back().pose.position == new_path.poses.back().pose.position;
}

bool PathLengthChecker::isRobotInGoalProximity(
  nav_msgs::msg::Path & old_path,
  double & prox_leng)
{
  return nav2_util::geometry_utils::calculate_path_length(old_path, 0) < prox_leng;
}

bool PathLengthChecker::isNewPathLonger(
  nav_msgs::msg::Path & new_path,
  nav_msgs::msg::Path & old_path,
  double & length_factor,
  double & abs_length)
{
  double new_len = nav2_util::geometry_utils::calculate_path_length(new_path, 0);
  double old_len = nav2_util::geometry_utils::calculate_path_length(old_path, 0);
  RCLCPP_DEBUG(node_->get_logger(), "New path length: %f, Old path length: %f", new_len, old_len);
  return (new_len > length_factor * old_len) &&
         ((new_len - old_len) > abs_length);
}

bool PathLengthChecker::arePathConditionsMet()
{
  return isPathUpdated(new_path_, old_path_) && 
         isRobotInGoalProximity(old_path_, prox_len_) &&
         isNewPathLonger(new_path_, old_path_, length_factor_, abs_length_);
}

void PathLengthChecker::resetWaitingState()
{
  is_waiting_ = false;
  longer_path_detected_ = false;
  wait_start_time_ = std::chrono::steady_clock::time_point{};
  last_check_time_ = std::chrono::steady_clock::time_point{};
}

bool PathLengthChecker::isWaitTimeoutExceeded()
{
  if (!is_waiting_) {
    return false;
  }
  
  auto current_time = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
    current_time - wait_start_time_).count();
  
  return elapsed >= wait_timeout_;
}

inline BT::NodeStatus PathLengthChecker::tick()
{
  getInput("path", new_path_);
  getInput("prox_len", prox_len_);
  getInput("length_factor", length_factor_);
  getInput("abs_length", abs_length_);
  getInput("wait_timeout", wait_timeout_);
  getInput("check_frequency", check_frequency_);
  getInput("enable_waiting", enable_waiting_);

  // Handle goal changes and first time execution
  if (first_time_ == false) {
    if (old_path_.poses.empty() || new_path_.poses.empty() ||
      old_path_.poses.back().pose != new_path_.poses.back().pose)
    {
      first_time_ = true;
      resetWaitingState();
    }
  }

  setStatus(BT::NodeStatus::RUNNING);

  // Check current path conditions
  bool conditions_met = arePathConditionsMet() && !first_time_;

  // Log current status
  RCLCPP_DEBUG(
    node_->get_logger(), 
    "PathLengthChecker - Path updated: %s, In goal proximity: %s, New path longer: %s, Waiting: %s",
    isPathUpdated(new_path_, old_path_) ? "true" : "false",
    isRobotInGoalProximity(old_path_, prox_len_) ? "true" : "false",
    isNewPathLonger(new_path_, old_path_, length_factor_, abs_length_) ? "true" : "false",
    is_waiting_ ? "true" : "false");

  // If waiting is disabled, behave like original PathLongerOnApproach
  if (!enable_waiting_) {
    if (conditions_met) {
      const BT::NodeStatus child_state = child_node_->executeTick();
      switch (child_state) {
        case BT::NodeStatus::RUNNING:
          return child_state;
        case BT::NodeStatus::SUCCESS:
        case BT::NodeStatus::FAILURE:
          old_path_ = new_path_;
          resetChild();
          return child_state;
        default:
          old_path_ = new_path_;
          return BT::NodeStatus::FAILURE;
      }
    }
    old_path_ = new_path_;
    first_time_ = false;
    return BT::NodeStatus::SUCCESS;
  }

  // Waiting mode logic
  auto current_time = std::chrono::steady_clock::now();
  
  if (conditions_met) {
    if (!longer_path_detected_) {
      RCLCPP_INFO(node_->get_logger(), "PathLengthChecker: Longer path detected!");
      longer_path_detected_ = true;
    }
    
    // Execute child immediately when longer path is detected
    const BT::NodeStatus child_state = child_node_->executeTick();
    switch (child_state) {
      case BT::NodeStatus::RUNNING:
        return child_state;
      case BT::NodeStatus::SUCCESS:
      case BT::NodeStatus::FAILURE:
        old_path_ = new_path_;
        resetChild();
        resetWaitingState();
        return child_state;
      default:
        old_path_ = new_path_;
        resetWaitingState();
        return BT::NodeStatus::FAILURE;
    }
  }
  
  // Check if we should start waiting (robot is in proximity but path is not longer yet)
  bool should_wait = isRobotInGoalProximity(old_path_, prox_len_) && !first_time_;
  
  if (should_wait && !is_waiting_) {
    RCLCPP_INFO(
      node_->get_logger(), 
      "PathLengthChecker: Starting to wait for longer path (timeout: %.1fs)", 
      wait_timeout_);
    is_waiting_ = true;
    wait_start_time_ = current_time;
    last_check_time_ = current_time;
  }
  
  // If we're waiting, check timeout and frequency
  if (is_waiting_) {
    // Check if timeout exceeded
    if (isWaitTimeoutExceeded()) {
      RCLCPP_WARN(
        node_->get_logger(), 
        "PathLengthChecker: Wait timeout exceeded (%.1fs), giving up", 
        wait_timeout_);
      resetWaitingState();
      old_path_ = new_path_;
      first_time_ = false;
      return BT::NodeStatus::SUCCESS;
    }
    
    // Control check frequency
    auto time_since_last_check = std::chrono::duration_cast<std::chrono::duration<double>>(
      current_time - last_check_time_).count();
    
    if (time_since_last_check >= (1.0 / check_frequency_)) {
      last_check_time_ = current_time;
      auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
        current_time - wait_start_time_).count();
      
      RCLCPP_DEBUG(
        node_->get_logger(), 
        "PathLengthChecker: Waiting for longer path... (%.1fs/%.1fs)", 
        elapsed, wait_timeout_);
    }
    
    // Keep running while waiting
    return BT::NodeStatus::RUNNING;
  }

  old_path_ = new_path_;
  first_time_ = false;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PathLengthChecker>("PathLengthChecker");
}
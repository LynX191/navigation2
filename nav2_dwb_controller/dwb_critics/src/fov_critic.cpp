#include "dwb_critics/fov_critic.hpp"

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "angles/angles.h"
#include "dwb_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dwb_critics::FOVCritic, dwb_core::TrajectoryCritic)

namespace dwb_critics
{

void FOVCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"FOVCritic: failed to lock node"};
  }
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  const std::string prefix = dwb_plugin_name_ + "." + name_;

  nav2_util::declare_parameter_if_not_declared(
    node, prefix + ".camera_yaws",
    rclcpp::ParameterValue(std::vector<double>{0.0, 2.0944, -2.0944}));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + ".camera_hfovs",
    rclcpp::ParameterValue(std::vector<double>{1.2217, 1.2217, 1.2217}));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + ".min_check_distance", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + ".check_every_nth_pose", rclcpp::ParameterValue(2));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + ".verbose_debug", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
  node, prefix + ".decouple_translate_rotate", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + ".max_rotation_while_translating", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
  node, prefix + ".forward_translation_cone", rclcpp::ParameterValue(0.35));
  nav2_util::declare_parameter_if_not_declared(
  node, prefix + ".translation_fov_margin", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
  node, prefix + ".translation_half_cone", rclcpp::ParameterValue(0.175));

  node->get_parameter(prefix + ".camera_yaws", camera_yaws_);
  node->get_parameter(prefix + ".camera_hfovs", camera_hfovs_);
  node->get_parameter(prefix + ".min_check_distance", min_check_distance_);
  node->get_parameter(prefix + ".check_every_nth_pose", check_every_nth_pose_);
  node->get_parameter(prefix + ".verbose_debug", verbose_debug_);
  node->get_parameter(prefix + ".decouple_translate_rotate", decouple_translate_rotate_);
  node->get_parameter(prefix + ".max_rotation_while_translating", max_rotation_while_translating_);
  node->get_parameter(prefix + ".forward_translation_cone", forward_translation_cone_);
  node->get_parameter(prefix + ".translation_fov_margin", translation_fov_margin_);
  node->get_parameter(prefix + ".translation_half_cone", translation_half_cone_);
  if (camera_yaws_.size() != camera_hfovs_.size() || camera_yaws_.empty()) {
    throw std::runtime_error{
      "FOVCritic: camera_yaws and camera_hfovs must be same length and non-empty"};
  }
  if (check_every_nth_pose_ < 1) check_every_nth_pose_ = 1;

  // Print configured sectors so wrong yaws/HFOVs are obvious
  std::ostringstream oss;
  oss << "FOVCritic init: " << camera_yaws_.size() << " cameras";
  for (size_t i = 0; i < camera_yaws_.size(); ++i) {
    const double half = camera_hfovs_[i] * 0.5;
    oss << "\n  cam[" << i << "] yaw=" << rad2deg(camera_yaws_[i]) << "deg "
        << "HFOV=" << rad2deg(camera_hfovs_[i]) << "deg "
        << "covers [" << rad2deg(camera_yaws_[i] - half) << ", "
        << rad2deg(camera_yaws_[i] + half) << "] (in base)";
  }
  oss << "\n  min_check_distance=" << min_check_distance_ << " m, "
      << "every_nth=" << check_every_nth_pose_
      << ", verbose=" << (verbose_debug_ ? "true" : "false");
  RCLCPP_INFO(logger_, "%s", oss.str().c_str());
}

bool FOVCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D & /*vel*/,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D & global_plan)
{
  // ---- Summary of previous cycle ----
  if (total_count_ > 0) {
    std::ostringstream oss;
    oss << "FOVCritic last cycle: total=" << total_count_
        << " accepted=" << accepted_count_
        << " rejected=" << rejected_count_
        << " inplace_skipped=" << inplace_count_;
    if (!sample_rejected_dirs_.empty()) {
      oss << " | rejected motion dirs (deg in base): ";
      for (double d : sample_rejected_dirs_) {
        oss << static_cast<int>(rad2deg(d)) << " ";
      }
    }
    RCLCPP_INFO(logger_, "%s", oss.str().c_str());
  }

  start_pose_ = pose;
  total_count_ = accepted_count_ = rejected_count_ = inplace_count_ = 0;
  sample_rejected_dirs_.clear();

  // ---- Diagnostic: where does the path want us to go? ----
  RCLCPP_INFO(
    logger_,
    "FOVCritic prepare: robot=(%.2f,%.2f,%.0fdeg) goal=(%.2f,%.2f,%.0fdeg) "
    "plan_poses=%zu",
    pose.x, pose.y, rad2deg(pose.theta),
    goal.x, goal.y, rad2deg(goal.theta), global_plan.poses.size());

  if (!global_plan.poses.empty()) {
    // Lookahead point ~1m along the plan (or last pose if shorter)
    double accumulated = 0.0;
    size_t idx = 0;
    for (size_t i = 1; i < global_plan.poses.size(); ++i) {
      accumulated += std::hypot(
        global_plan.poses[i].x - global_plan.poses[i - 1].x,
        global_plan.poses[i].y - global_plan.poses[i - 1].y);
      idx = i;
      if (accumulated >= 1.0) break;
    }
    const auto & lookahead = global_plan.poses[idx];
    const double dx = lookahead.x - pose.x;
    const double dy = lookahead.y - pose.y;
    if (std::hypot(dx, dy) > 1e-3) {
      const double dir_world = std::atan2(dy, dx);
      const double dir_base = angles::normalize_angle(dir_world - pose.theta);
      const bool covered = isDirectionCovered(dir_base);
      RCLCPP_INFO(
        logger_,
        "  path lookahead dir (in base): %.0f deg -> %s",
        rad2deg(dir_base),
        covered ? "COVERED" : "BLIND (robot must rotate or path unreachable)");
    }
  }
  return true;
}

bool FOVCritic::isDirectionCovered(double angle_in_base) const
{
  for (size_t i = 0; i < camera_yaws_.size(); ++i) {
    const double diff = std::fabs(
      angles::shortest_angular_distance(camera_yaws_[i], angle_in_base));
    if (diff <= camera_hfovs_[i] * 0.5) {
      return true;
    }
  }
  return false;
}

bool FOVCritic::isDirectionCoveredWithMargin(
  double angle_in_base, double margin) const
{
  for (size_t i = 0; i < camera_yaws_.size(); ++i) {
    const double half = camera_hfovs_[i] * 0.5 - margin;
    if (half <= 0.0) continue;
    const double diff = std::fabs(
      angles::shortest_angular_distance(camera_yaws_[i], angle_in_base));
    if (diff <= half) return true;
  }
  return false;
}

bool FOVCritic::isDirectionInTranslationCone(double angle_in_base) const
{
  for (size_t i = 0; i < camera_yaws_.size(); ++i) {
    const double diff = std::fabs(
      angles::shortest_angular_distance(camera_yaws_[i], angle_in_base));
    if (diff <= translation_half_cone_) return true;
  }
  return false;
}

double FOVCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  total_count_++;

  if (traj.poses.empty()) {
    accepted_count_++;
    return 0.0;
  }

  const auto & last = traj.poses.back();
  const double total_disp = std::hypot(
    last.x - start_pose_.x, last.y - start_pose_.y);

  if (total_disp < min_check_distance_) {
    inplace_count_++;
    return 0.0;
  }
  if (decouple_translate_rotate_) {
    const double dtheta = std::fabs(
      angles::shortest_angular_distance(start_pose_.theta, last.theta));

    if (dtheta > max_rotation_while_translating_) {
      rejected_count_++;
      throw dwb_core::IllegalTrajectoryException(
        name_, "Mixed translate+rotate not allowed");
    }

    const double dx = last.x - start_pose_.x;
    const double dy = last.y - start_pose_.y;
    const double motion_dir_world = std::atan2(dy, dx);
    const double motion_dir_in_base = angles::normalize_angle(
      motion_dir_world - start_pose_.theta);

    if (!isDirectionInTranslationCone(motion_dir_in_base)) {
      rejected_count_++;
      throw dwb_core::IllegalTrajectoryException(
        name_, "Translation direction not aligned with any camera center");
    }
    
    // Translation must point into one of the camera FOVs (with margin)
    if (!isDirectionCoveredWithMargin(motion_dir_in_base, translation_fov_margin_)) {
      rejected_count_++;
      throw dwb_core::IllegalTrajectoryException(
        name_, "Translation direction not aligned with any camera FOV");
    }
  }
  for (size_t i = 0; i < traj.poses.size();
       i += static_cast<size_t>(check_every_nth_pose_))
  {
    const auto & p = traj.poses[i];
    const double dx = p.x - start_pose_.x;
    const double dy = p.y - start_pose_.y;
    if (std::hypot(dx, dy) < min_check_distance_) continue;

    const double motion_dir_world = std::atan2(dy, dx);
    const double motion_dir_in_base = angles::normalize_angle(
      motion_dir_world - p.theta);

    if (verbose_debug_) {
      RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 500,
        "  traj#%u pose#%zu world(%.2f,%.2f,%.0fdeg) "
        "motion_world=%.0fdeg motion_base=%.0fdeg covered=%s",
        total_count_, i, p.x, p.y, rad2deg(p.theta),
        rad2deg(motion_dir_world), rad2deg(motion_dir_in_base),
        isDirectionCovered(motion_dir_in_base) ? "yes" : "NO");
    }

    if (!isDirectionCovered(motion_dir_in_base)) {
      rejected_count_++;
      if (sample_rejected_dirs_.size() < 5) {
        sample_rejected_dirs_.push_back(motion_dir_in_base);
      }
      throw dwb_core::IllegalTrajectoryException(
        name_, "Trajectory enters camera blind sector");
    }
  }

  accepted_count_++;
  return 0.0;
}

}  // namespace dwb_critics
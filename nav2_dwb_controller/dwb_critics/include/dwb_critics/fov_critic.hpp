#ifndef DWB_CRITICS__FOV_CRITIC_HPP_
#define DWB_CRITICS__FOV_CRITIC_HPP_

#include <cmath>
#include <vector>

#include "dwb_core/trajectory_critic.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dwb_critics
{

class FOVCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override;
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal,
    const nav_2d_msgs::msg::Path2D & global_plan) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;

private:
  bool isDirectionCovered(double angle_in_base) const;
  static double rad2deg(double r) { return r * 180.0 / M_PI; }

  // params
  std::vector<double> camera_yaws_;
  std::vector<double> camera_hfovs_;
  double min_check_distance_{0.15};
  int check_every_nth_pose_{2};
  bool verbose_debug_{false};

  geometry_msgs::msg::Pose2D start_pose_;
  double forward_translation_cone_{0.35};  // rad, ~20deg
  bool decouple_translate_rotate_{true};
  double max_rotation_while_translating_{0.05};  // rad, ~3 deg
  double translation_half_cone_{0.175};   // rad, ±10deg around each camera center
  bool isDirectionInTranslationCone(double angle_in_base) const;
  // per-cycle debug counters
  unsigned int total_count_{0};
  unsigned int accepted_count_{0};
  unsigned int rejected_count_{0};
  unsigned int inplace_count_{0};
  std::vector<double> sample_rejected_dirs_;
  bool isDirectionCoveredWithMargin(double angle_in_base, double margin) const;
  double translation_fov_margin_{0.2};  // rad, shrink each FOV by this for translation
  rclcpp::Logger logger_{rclcpp::get_logger("FOVCritic")};
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace dwb_critics
#endif
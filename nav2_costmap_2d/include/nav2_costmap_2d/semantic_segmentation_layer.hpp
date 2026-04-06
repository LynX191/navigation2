/*********************************************************************
 *
 * Software License Agreement
 *
 *  Copyright (c) 2026, robot.com
 *  All rights reserved.
 *
 *  (License text unchanged — see header)
 *
 * Authors: Pedro Gonzalez (pedro@robot.com)
 *          Johan Solarte (jsolarte@robot.com)
 *********************************************************************/
#ifndef SEMANTIC_SEGMENTATION_LAYER_HPP_
#define SEMANTIC_SEGMENTATION_LAYER_HPP_

#include <functional>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.hpp"
#include "message_filters/time_synchronizer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/segmentation_buffer.hpp"
#include "nav2_util/node_utils.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/message_filter.hpp"
#include "vision_msgs/msg/label_info.hpp"

namespace nav2_costmap_2d {

/**
 * @class SemanticSegmentationLayer
 * @brief Populates the 2D costmap from semantic segmentation + aligned pointclouds.
 *
 * Thread-safety notes
 * -------------------
 * updateBounds / updateCosts run in the costmap map-update thread and hold
 * the costmap mutex (getMutex()) for their full duration.
 *
 * Subscription callbacks (syncSegm*, labelinfoCb) run in the ROS executor
 * thread(s).  Every callback acquires buffer->getMutex() before touching any
 * buffer state, ensuring no concurrent access to segmentation_cost_multimap_
 * or temporal_tile_map_.
 */
class SemanticSegmentationLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  SemanticSegmentationLayer();
  virtual ~SemanticSegmentationLayer() {}

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                           int min_i, int min_j, int max_i, int max_j);
  virtual void reset();
  virtual void onFootprintChanged();
  virtual bool isClearable() { return true; }
  virtual void activate();
  virtual void deactivate();

  bool getSegmentationTileMaps(
    std::vector<std::pair<SegmentationTileMap::SharedPtr,
                          SegmentationBuffer::SharedPtr>>& segmentation_tile_maps);

  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

private:
  void syncSegmPointcloudCb(
    const std::shared_ptr<const sensor_msgs::msg::Image>&        segmentation,
    const std::shared_ptr<const sensor_msgs::msg::PointCloud2>&  pointcloud,
    const std::shared_ptr<nav2_costmap_2d::SegmentationBuffer>&  buffer);

  void syncSegmConfPointcloudCb(
    const std::shared_ptr<const sensor_msgs::msg::Image>&        segmentation,
    const std::shared_ptr<const sensor_msgs::msg::Image>&        confidence,
    const std::shared_ptr<const sensor_msgs::msg::PointCloud2>&  pointcloud,
    const std::shared_ptr<nav2_costmap_2d::SegmentationBuffer>&  buffer);

  void labelinfoCb(
    const std::shared_ptr<const vision_msgs::msg::LabelInfo>&    label_info,
    const std::shared_ptr<nav2_costmap_2d::SegmentationBuffer>&  buffer);

  // Subscriptions
  using ImageSub    = message_filters::Subscriber<sensor_msgs::msg::Image,
                                                  rclcpp_lifecycle::LifecycleNode>;
  using LabelSub    = message_filters::Subscriber<vision_msgs::msg::LabelInfo,
                                                  rclcpp_lifecycle::LifecycleNode>;
  using PC2Sub      = message_filters::Subscriber<sensor_msgs::msg::PointCloud2,
                                                  rclcpp_lifecycle::LifecycleNode>;
  using SyncSegmPC  = message_filters::TimeSynchronizer<sensor_msgs::msg::Image,
                                                        sensor_msgs::msg::PointCloud2>;
  using SyncSegmConfPC = message_filters::TimeSynchronizer<sensor_msgs::msg::Image,
                                                           sensor_msgs::msg::Image,
                                                           sensor_msgs::msg::PointCloud2>;

  std::vector<std::shared_ptr<ImageSub>>       semantic_segmentation_subs_;
  std::vector<std::shared_ptr<ImageSub>>       semantic_segmentation_confidence_subs_;
  std::vector<std::shared_ptr<LabelSub>>       label_info_subs_;
  std::vector<std::shared_ptr<PC2Sub>>         pointcloud_subs_;
  std::vector<std::shared_ptr<SyncSegmPC>>     segm_pc_notifiers_;
  std::vector<std::shared_ptr<SyncSegmConfPC>> segm_conf_pc_notifiers_;
  std::vector<std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>>
    pointcloud_tf_subs_;

  std::map<std::string,
           std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>>>
    proc_pointcloud_pubs_map_;

  std::vector<std::shared_ptr<nav2_costmap_2d::SegmentationBuffer>> segmentation_buffers_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  std::string global_frame_;
  std::string topics_string_;
  std::map<std::string, uint8_t> class_map_;

  bool rolling_window_ = false;
  bool was_reset_      = false;
  int  combination_method_ = 1;
};

}  // namespace nav2_costmap_2d
#endif  // SEMANTIC_SEGMENTATION_LAYER_HPP_
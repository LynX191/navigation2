/*********************************************************************
 *
 * Software License Agreement
 *
 *  Copyright (c) 2026, robot.com
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of robot.com nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Pedro Gonzalez (pedro@robot.com)
 *          Johan Solarte (jsolarte@robot.com)
 *********************************************************************/
#include "nav2_costmap_2d/semantic_segmentation_layer.hpp"

#include <algorithm>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "rmw/qos_profiles.h"

using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_costmap_2d {

SemanticSegmentationLayer::SemanticSegmentationLayer() {}

void SemanticSegmentationLayer::onInitialize()
{
  current_ = true;
  was_reset_ = false;
  auto node = node_.lock();
  if (!node)
  {
    throw std::runtime_error{"Failed to lock node"};
  }
  std::string segmentation_topic, confidence_topic, pointcloud_topic, labels_topic;
  std::vector<std::string> class_types_string;
  double max_obstacle_distance, min_obstacle_distance, observation_keep_time, transform_tolerance,
    expected_update_rate, tile_map_decay_time;
  bool track_unknown_space, visualize_tile_map;

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("combination_method", rclcpp::ParameterValue(1));
  declareParameter("observation_sources", rclcpp::ParameterValue(std::string("")));
  declareParameter("publish_debug_topics", rclcpp::ParameterValue(false));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "combination_method", combination_method_);
  node->get_parameter("track_unknown_space", track_unknown_space);
  node->get_parameter("transform_tolerance", transform_tolerance);

  global_frame_ = layered_costmap_->getGlobalFrameID();
  rolling_window_ = layered_costmap_->isRolling();

  if (track_unknown_space) {
    default_value_ = NO_INFORMATION;
  } else {
    default_value_ = nav2_costmap_2d::FREE_SPACE;
  }

  matchSize();

  node->get_parameter(name_ + "." + "observation_sources", topics_string_);

  std::stringstream ss(topics_string_);
  std::string source;

  while (ss >> source) {
    declareParameter(source + "." + "segmentation_topic", rclcpp::ParameterValue(""));
    declareParameter(source + "." + "confidence_topic", rclcpp::ParameterValue(""));
    declareParameter(source + "." + "labels_topic", rclcpp::ParameterValue(""));
    declareParameter(source + "." + "pointcloud_topic", rclcpp::ParameterValue(""));
    declareParameter(source + "." + "observation_persistence", rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "expected_update_rate", rclcpp::ParameterValue(0.0));
    declareParameter(source + "." + "class_types", rclcpp::ParameterValue(std::vector<std::string>({})));
    declareParameter(source + "." + "max_obstacle_distance", rclcpp::ParameterValue(5.0));
    declareParameter(source + "." + "min_obstacle_distance", rclcpp::ParameterValue(0.3));
    declareParameter(source + "." + "tile_map_decay_time", rclcpp::ParameterValue(5.0));
    declareParameter(source + "." + "visualize_tile_map", rclcpp::ParameterValue(false));
    declareParameter(source + "." + "use_cost_selection", rclcpp::ParameterValue(true));

    node->get_parameter(name_ + "." + source + "." + "segmentation_topic", segmentation_topic);
    node->get_parameter(name_ + "." + source + "." + "confidence_topic", confidence_topic);
    node->get_parameter(name_ + "." + source + "." + "labels_topic", labels_topic);
    node->get_parameter(name_ + "." + source + "." + "pointcloud_topic", pointcloud_topic);
    node->get_parameter(name_ + "." + source + "." + "observation_persistence", observation_keep_time);
    node->get_parameter(name_ + "." + source + "." + "expected_update_rate", expected_update_rate);
    node->get_parameter(name_ + "." + source + "." + "class_types", class_types_string);
    node->get_parameter(name_ + "." + source + "." + "max_obstacle_distance", max_obstacle_distance);
    node->get_parameter(name_ + "." + source + "." + "min_obstacle_distance", min_obstacle_distance);
    node->get_parameter(name_ + "." + source + "." + "tile_map_decay_time", tile_map_decay_time);
    node->get_parameter(name_ + "." + source + "." + "visualize_tile_map", visualize_tile_map);
    bool use_cost_selection = true;
    node->get_parameter(name_ + "." + source + "." + "use_cost_selection", use_cost_selection);

    if (class_types_string.empty())
    {
      RCLCPP_ERROR(logger_, "no class types defined for source %s. Segmentation plugin cannot work this way", source.c_str());
      exit(-1);
    }

    std::unordered_map<std::string, CostHeuristicParams> class_map;
    std::unordered_map<std::string, std::vector<std::string>> class_type_to_names;

    for (auto& class_type : class_types_string)
    {
      std::vector<std::string> classes_ids;
      declareParameter(source + "." + class_type + ".classes", rclcpp::ParameterValue(std::vector<std::string>({})));
      declareParameter(source + "." + class_type + ".base_cost", rclcpp::ParameterValue(0));
      declareParameter(source + "." + class_type + ".max_cost", rclcpp::ParameterValue(0));
      declareParameter(source + "." + class_type + ".mark_confidence", rclcpp::ParameterValue(0));
      declareParameter(source + "." + class_type + ".samples_to_max_cost", rclcpp::ParameterValue(0));
      declareParameter(source + "." + class_type + ".dominant_priority", rclcpp::ParameterValue(false));

      node->get_parameter(name_ + "." + source + "." + class_type + ".classes", classes_ids);
      if (classes_ids.empty())
      {
        RCLCPP_ERROR(logger_, "no classes defined on type %s", class_type.c_str());
        continue;
      }

      class_type_to_names[class_type] = classes_ids;

      CostHeuristicParams cost_params;
      node->get_parameter(name_ + "." + source + "." + class_type + ".base_cost", cost_params.base_cost);
      node->get_parameter(name_ + "." + source + "." + class_type + ".max_cost", cost_params.max_cost);
      node->get_parameter(name_ + "." + source + "." + class_type + ".mark_confidence", cost_params.mark_confidence);
      node->get_parameter(name_ + "." + source + "." + class_type + ".samples_to_max_cost", cost_params.samples_to_max_cost);
      node->get_parameter(name_ + "." + source + "." + class_type + ".dominant_priority", cost_params.dominant_priority);

      for (auto& class_id : classes_ids)
      {
        class_map.insert(std::pair<std::string, CostHeuristicParams>(class_id, cost_params));
      }
    }

    if (class_map.empty())
    {
      RCLCPP_ERROR(logger_, "No classes defined for source %s. Segmentation plugin cannot work this way", source.c_str());
      exit(-1);
    }

    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_;
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.depth = 50;

    rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> tl_sub_opt;
    tl_sub_opt.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    tl_sub_opt.callback_group = callback_group_;
    rmw_qos_profile_t tl_qos_profile = rmw_qos_profile_default;
    tl_qos_profile.depth = 5;
    tl_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    tl_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

    auto segmentation_buffer = std::make_shared<nav2_costmap_2d::SegmentationBuffer>(
      node, source, class_types_string, class_map, class_type_to_names, observation_keep_time,
      expected_update_rate, max_obstacle_distance, min_obstacle_distance, *tf_, global_frame_, "",
      tf2::durationFromSec(transform_tolerance), getResolution(), tile_map_decay_time,
      visualize_tile_map, use_cost_selection);

    segmentation_buffers_.push_back(segmentation_buffer);

    auto semantic_segmentation_sub =
      std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image, rclcpp_lifecycle::LifecycleNode>>(
        node, segmentation_topic, custom_qos_profile, sub_opt);
    semantic_segmentation_sub->unsubscribe();
    semantic_segmentation_subs_.push_back(semantic_segmentation_sub);

    auto label_info_sub = std::make_shared<message_filters::Subscriber<vision_msgs::msg::LabelInfo, rclcpp_lifecycle::LifecycleNode>>(
        node, labels_topic, tl_qos_profile, tl_sub_opt);
    label_info_sub->registerCallback(std::bind(&SemanticSegmentationLayer::labelinfoCb, this,
                                               std::placeholders::_1, segmentation_buffers_.back()));
    label_info_sub->unsubscribe();
    label_info_subs_.push_back(label_info_sub);

    auto pointcloud_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2, rclcpp_lifecycle::LifecycleNode>>(
      node, pointcloud_topic, custom_qos_profile, sub_opt);
    pointcloud_sub->unsubscribe();
    pointcloud_subs_.push_back(pointcloud_sub);

    auto pointcloud_tf_sub = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
      *pointcloud_subs_.back(), *tf_, global_frame_, 50,
      node->get_node_logging_interface(),
      node->get_node_clock_interface(),
      tf2::durationFromSec(transform_tolerance));
    pointcloud_tf_subs_.push_back(pointcloud_tf_sub);

    if (!confidence_topic.empty())
    {
      auto semantic_segmentation_confidence_sub =
        std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image, rclcpp_lifecycle::LifecycleNode>>(
          node, confidence_topic, custom_qos_profile, sub_opt);
      semantic_segmentation_confidence_sub->unsubscribe();
      semantic_segmentation_confidence_subs_.push_back(semantic_segmentation_confidence_sub);

      auto segm_conf_pc_sync =
        std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image,
                                                           sensor_msgs::msg::Image,
                                                           sensor_msgs::msg::PointCloud2>>(1000);
      segm_conf_pc_sync->connectInput(*semantic_segmentation_subs_.back(),
                                      *semantic_segmentation_confidence_subs_.back(),
                                      *pointcloud_tf_subs_.back());
      segm_conf_pc_sync->registerCallback(
        std::bind(&SemanticSegmentationLayer::syncSegmConfPointcloudCb, this,
                  std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                  segmentation_buffers_.back()));
      segm_conf_pc_notifiers_.push_back(segm_conf_pc_sync);
      RCLCPP_INFO(logger_, "Confidence is enabled for source %s", source.c_str());
    }
    else
    {
      RCLCPP_WARN(logger_, "Confidence topic was empty for source %s, not using segmentation confidence in that source", source.c_str());
      auto segm_pc_sync =
        std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image,
                                                           sensor_msgs::msg::PointCloud2>>(1000);
      segm_pc_sync->connectInput(*semantic_segmentation_subs_.back(), *pointcloud_tf_subs_.back());
      segm_pc_sync->registerCallback(
        std::bind(&SemanticSegmentationLayer::syncSegmPointcloudCb, this,
                  std::placeholders::_1, std::placeholders::_2,
                  segmentation_buffers_.back()));
      segm_pc_notifiers_.push_back(segm_pc_sync);
    }
  }

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&SemanticSegmentationLayer::dynamicParametersCallback, this, std::placeholders::_1));
}

void SemanticSegmentationLayer::updateBounds(double robot_x, double robot_y, double /*robot_yaw*/,
                                             double* min_x, double* min_y, double* max_x,
                                             double* max_y)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }
  if (!enabled_) {
    return;
  }

  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(logger_, "Failed to lock node in updateBounds");
    return;
  }

  double current_time = node->now().seconds();
  if (current_time <= 0.0) {
    RCLCPP_WARN(logger_, "Invalid current time in updateBounds: %.3f", current_time);
    return;
  }

  // Iterate buffers directly without going through getSegmentationTileMaps().
  // The old two-step lock/unlock in getSegmentationTileMaps() followed by a
  // second lock() here created a window where the tile_map pointer was used
  // outside the lock (data race) and, when a TF exception fired between
  // buffer->lock() and buffer->unlock(), the mutex was permanently leaked
  // causing an infinite deadlock on the next mapUpdateLoop cycle.
  //
  // Fix: use std::unique_lock (RAII) so the mutex is always released even
  // when an exception propagates out of the loop body.
  for (auto& buffer : segmentation_buffers_)
  {
    // RAII lock — released automatically on scope exit, including exceptions
    std::unique_lock<std::recursive_mutex> buffer_lock(buffer->getMutex());

    SegmentationTileMap::SharedPtr tile_map = buffer->getSegmentationTileMap();
    if (!tile_map) {
      continue;
    }

    // Purge stale observations before computing costs so the costmap
    // accurately reflects the current state after decay.
    tile_map->purgeOldObservations(current_time);

    for (auto& tile : *tile_map)
    {
      if (tile.second.empty()) {
        continue;
      }

      TileWorldXY tile_world_coords = tile_map->indexToWorld(tile.first.x, tile.first.y);
      TemporalObservationQueue& obs_queue = tile.second;

      unsigned int mx, my;
      if (!worldToMap(tile_world_coords.x, tile_world_coords.y, mx, my)) {
        RCLCPP_DEBUG(logger_, "Computing map coords failed");
        continue;
      }

      unsigned int index = getIndex(mx, my);
      CostHeuristicParams cost_params = buffer->getCostForClassId(obs_queue.getClassId());

      if (static_cast<int>(obs_queue.size()) >= cost_params.samples_to_max_cost &&
          obs_queue.getConfidenceSum() / obs_queue.size() > cost_params.mark_confidence)
      {
        costmap_[index] = cost_params.max_cost;
      } else {
        costmap_[index] = cost_params.base_cost;
      }

      touch(tile_world_coords.x, tile_world_coords.y, min_x, min_y, max_x, max_y);
    }
    // buffer_lock destructor releases the mutex here — guaranteed even on exception
  }

  current_ = true;
}

void SemanticSegmentationLayer::onFootprintChanged()
{
  RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"),
               "SemanticSegmentationLayer::onFootprintChanged(): num footprint points: %lu",
               layered_costmap_->getFootprint().size());
}

void SemanticSegmentationLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i,
                                            int min_j, int max_i, int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) {
    return;
  }

  if (!current_ && was_reset_) {
    was_reset_ = false;
    current_ = true;
  }
  if (!costmap_) {
    return;
  }

  switch (combination_method_)
  {
    case 0:  // Overwrite
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:  // Maximum
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:
      break;
  }
}

void SemanticSegmentationLayer::labelinfoCb(
  const std::shared_ptr<const vision_msgs::msg::LabelInfo>& label_info,
  const std::shared_ptr<nav2_costmap_2d::SegmentationBuffer>& buffer)
{
  buffer->createSegmentationCostMultimap(*label_info);
}

void SemanticSegmentationLayer::syncSegmPointcloudCb(
  const std::shared_ptr<const sensor_msgs::msg::Image>& segmentation,
  const std::shared_ptr<const sensor_msgs::msg::PointCloud2>& pointcloud,
  const std::shared_ptr<nav2_costmap_2d::SegmentationBuffer>& buffer)
{
  if (segmentation->width * segmentation->height != pointcloud->width * pointcloud->height)
  {
    RCLCPP_WARN(logger_,
                "Pointcloud and segmentation sizes are different, will not buffer message. "
                "segmentation->width:%u, segmentation->height:%u, "
                "pointcloud->width:%u, pointcloud->height:%u",
                segmentation->width, segmentation->height,
                pointcloud->width, pointcloud->height);
    return;
  }

  unsigned expected_array_size = segmentation->width * segmentation->height;
  if (segmentation->data.size() < expected_array_size)
  {
    RCLCPP_WARN(logger_,
                "segmentation arrays have wrong sizes: data->%lu, expected->%u. "
                "Will not buffer message",
                segmentation->data.size(), expected_array_size);
    return;
  }

  if (buffer->isClassIdCostMapEmpty())
  {
    RCLCPP_WARN(logger_,
                "Class map is empty because a labelinfo message has not been received "
                "for topic %s. Will not buffer message",
                buffer->getBufferSource().c_str());
    return;
  }

  // No confidence available: fill mask with max confidence so that
  // thresholding works purely on accumulated observation count.
  sensor_msgs::msg::Image conf_mask = *segmentation;
  std::fill(conf_mask.data.begin(), conf_mask.data.end(), 255);

  // RAII lock — released automatically on scope exit, including exceptions
  std::unique_lock<std::recursive_mutex> buffer_lock(buffer->getMutex());
  buffer->bufferSegmentation(*pointcloud, *segmentation, conf_mask);
}

void SemanticSegmentationLayer::syncSegmConfPointcloudCb(
  const std::shared_ptr<const sensor_msgs::msg::Image>& segmentation,
  const std::shared_ptr<const sensor_msgs::msg::Image>& confidence,
  const std::shared_ptr<const sensor_msgs::msg::PointCloud2>& pointcloud,
  const std::shared_ptr<nav2_costmap_2d::SegmentationBuffer>& buffer)
{
  if (segmentation->width * segmentation->height != pointcloud->width * pointcloud->height)
  {
    RCLCPP_WARN(logger_,
                "Pointcloud and segmentation sizes are different, will not buffer message. "
                "segmentation->width:%u, segmentation->height:%u, "
                "pointcloud->width:%u, pointcloud->height:%u",
                segmentation->width, segmentation->height,
                pointcloud->width, pointcloud->height);
    return;
  }

  unsigned expected_array_size = segmentation->width * segmentation->height;
  if (segmentation->data.size() < expected_array_size)
  {
    RCLCPP_WARN(logger_,
                "segmentation arrays have wrong sizes: data->%lu, expected->%u. "
                "Will not buffer message",
                segmentation->data.size(), expected_array_size);
    return;
  }

  if (buffer->isClassIdCostMapEmpty())
  {
    RCLCPP_WARN(logger_,
                "Class map is empty because a labelinfo message has not been received "
                "for topic %s. Will not buffer message",
                buffer->getBufferSource().c_str());
    return;
  }

  // RAII lock — released automatically on scope exit, including exceptions
  std::unique_lock<std::recursive_mutex> buffer_lock(buffer->getMutex());
  buffer->bufferSegmentation(*pointcloud, *segmentation, *confidence);
}

void SemanticSegmentationLayer::reset()
{
  resetMaps();
  current_ = false;
  was_reset_ = true;
}

// NOTE: This function is retained for any external callers but is no longer
// used by updateBounds(). The previous implementation released the buffer lock
// before returning tile_map pointers to the caller, creating a data race window.
// If you call this function, you must hold each buffer's mutex for the entire
// duration that the returned tile_map shared_ptr is accessed.
bool SemanticSegmentationLayer::getSegmentationTileMaps(
  std::vector<std::pair<SegmentationTileMap::SharedPtr,
                        SegmentationBuffer::SharedPtr>>& segmentation_tile_maps)
{
  for (unsigned int i = 0; i < segmentation_buffers_.size(); ++i) {
    // Lock while retrieving the tile_map pointer. Callers are responsible
    // for holding buffer->getMutex() across the lifetime of the tile_map use.
    std::unique_lock<std::recursive_mutex> buffer_lock(segmentation_buffers_[i]->getMutex());
    SegmentationTileMap::SharedPtr tile_map = segmentation_buffers_[i]->getSegmentationTileMap();
    segmentation_tile_maps.emplace_back(std::make_pair(tile_map, segmentation_buffers_[i]));
    // WARNING: lock released here — tile_map data is only safe to read while
    // you hold segmentation_buffers_[i]->getMutex() yourself.
  }
  return true;
}

rcl_interfaces::msg::SetParametersResult
SemanticSegmentationLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  auto result = rcl_interfaces::msg::SetParametersResult();

  for (auto parameter : parameters) {
    const auto& type = parameter.get_type();
    const auto& name = parameter.get_name();

    if (type == rclcpp::ParameterType::PARAMETER_BOOL) {
      if (name == name_ + "." + "enabled") {
        enabled_ = parameter.as_bool();
      }
    }

    std::stringstream ss(topics_string_);
    std::string source;
    while (ss >> source) {
      if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (name == name_ + "." + source + "." + "max_obstacle_distance") {
          for (auto& buffer : segmentation_buffers_) {
            if (buffer->getBufferSource() == source) {
              buffer->setMaxObstacleDistance(parameter.as_double());
            }
          }
        } else if (name == name_ + "." + source + "." + "min_obstacle_distance") {
          for (auto& buffer : segmentation_buffers_) {
            if (buffer->getBufferSource() == source) {
              buffer->setMinObstacleDistance(parameter.as_double());
            }
          }
        }
      } else if (type == rclcpp::ParameterType::PARAMETER_INTEGER) {
        for (auto& buffer : segmentation_buffers_) {
          if (buffer->getBufferSource() == source) {
            for (auto& class_type : buffer->getClassTypes()) {
              auto class_names_for_type = buffer->getClassNamesForType(class_type);

              if (name == name_ + "." + source + "." + class_type + "." + "base_cost") {
                for (auto& class_name : class_names_for_type) {
                  CostHeuristicParams cost_params = buffer->getCostForClassName(class_name);
                  cost_params.base_cost = parameter.as_int();
                  buffer->updateClassMap(class_name, cost_params);
                }
              }
              if (name == name_ + "." + source + "." + class_type + "." + "max_cost") {
                for (auto& class_name : class_names_for_type) {
                  CostHeuristicParams cost_params = buffer->getCostForClassName(class_name);
                  cost_params.max_cost = parameter.as_int();
                  buffer->updateClassMap(class_name, cost_params);
                }
              }
              if (name == name_ + "." + source + "." + class_type + "." + "mark_confidence") {
                for (auto& class_name : class_names_for_type) {
                  CostHeuristicParams cost_params = buffer->getCostForClassName(class_name);
                  cost_params.mark_confidence = parameter.as_int();
                  buffer->updateClassMap(class_name, cost_params);
                }
              }
              if (name == name_ + "." + source + "." + class_type + "." + "samples_to_max_cost") {
                for (auto& class_name : class_names_for_type) {
                  CostHeuristicParams cost_params = buffer->getCostForClassName(class_name);
                  cost_params.samples_to_max_cost = parameter.as_int();
                  buffer->updateClassMap(class_name, cost_params);
                }
              }
              if (name == name_ + "." + source + "." + class_type + ".dominant_priority") {
                for (auto& class_name : class_names_for_type) {
                  CostHeuristicParams cost_params = buffer->getCostForClassName(class_name);
                  cost_params.dominant_priority = parameter.as_bool();
                  buffer->updateClassMap(class_name, cost_params);
                }
              }
            }
          }
        }
      }
    }
  }

  result.successful = true;
  return result;
}

void SemanticSegmentationLayer::activate()
{
  for (unsigned int i = 0; i < semantic_segmentation_subs_.size(); ++i) {
    if (semantic_segmentation_subs_[i] != NULL) {
      semantic_segmentation_subs_[i]->subscribe();
    }
  }
  for (unsigned int i = 0; i < semantic_segmentation_confidence_subs_.size(); ++i) {
    if (semantic_segmentation_confidence_subs_[i] != NULL) {
      semantic_segmentation_confidence_subs_[i]->subscribe();
    }
  }
  for (unsigned int i = 0; i < label_info_subs_.size(); ++i) {
    if (label_info_subs_[i] != NULL) {
      label_info_subs_[i]->subscribe();
    }
  }
  for (unsigned int i = 0; i < pointcloud_subs_.size(); ++i) {
    if (pointcloud_subs_[i] != NULL) {
      pointcloud_subs_[i]->subscribe();
    }
  }
}

void SemanticSegmentationLayer::deactivate()
{
  for (unsigned int i = 0; i < semantic_segmentation_subs_.size(); ++i) {
    if (semantic_segmentation_subs_[i] != NULL) {
      semantic_segmentation_subs_[i]->unsubscribe();
    }
  }
  for (unsigned int i = 0; i < semantic_segmentation_confidence_subs_.size(); ++i) {
    if (semantic_segmentation_confidence_subs_[i] != NULL) {
      semantic_segmentation_confidence_subs_[i]->unsubscribe();
    }
  }
  for (unsigned int i = 0; i < label_info_subs_.size(); ++i) {
    if (label_info_subs_[i] != NULL) {
      label_info_subs_[i]->unsubscribe();
    }
  }
  for (unsigned int i = 0; i < pointcloud_subs_.size(); ++i) {
    if (pointcloud_subs_[i] != NULL) {
      pointcloud_subs_[i]->unsubscribe();
    }
  }
}

}  // namespace nav2_costmap_2d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::SemanticSegmentationLayer, nav2_costmap_2d::Layer)
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
  current_   = true;
  was_reset_ = false;

  auto node = node_.lock();
  if (!node) throw std::runtime_error{"Failed to lock node"};

  std::string segmentation_topic, confidence_topic, pointcloud_topic, labels_topic;
  std::vector<std::string> class_types_string;
  double max_obstacle_distance, min_obstacle_distance, observation_keep_time,
         transform_tolerance, expected_update_rate, tile_map_decay_time;
  bool track_unknown_space, visualize_tile_map;

  declareParameter("enabled",              rclcpp::ParameterValue(true));
  declareParameter("combination_method",   rclcpp::ParameterValue(1));
  declareParameter("observation_sources",  rclcpp::ParameterValue(std::string("")));
  declareParameter("publish_debug_topics", rclcpp::ParameterValue(false));

  node->get_parameter(name_ + ".enabled",            enabled_);
  node->get_parameter(name_ + ".combination_method", combination_method_);
  node->get_parameter("track_unknown_space",          track_unknown_space);
  node->get_parameter("transform_tolerance",          transform_tolerance);

  global_frame_   = layered_costmap_->getGlobalFrameID();
  rolling_window_ = layered_costmap_->isRolling();
  default_value_  = track_unknown_space ? NO_INFORMATION : nav2_costmap_2d::FREE_SPACE;

  matchSize();

  node->get_parameter(name_ + ".observation_sources", topics_string_);

  std::stringstream ss(topics_string_);
  std::string source;

  while (ss >> source) {
    declareParameter(source + ".segmentation_topic",    rclcpp::ParameterValue(""));
    declareParameter(source + ".confidence_topic",      rclcpp::ParameterValue(""));
    declareParameter(source + ".labels_topic",          rclcpp::ParameterValue(""));
    declareParameter(source + ".pointcloud_topic",      rclcpp::ParameterValue(""));
    declareParameter(source + ".observation_persistence", rclcpp::ParameterValue(0.0));
    declareParameter(source + ".expected_update_rate",  rclcpp::ParameterValue(0.0));
    declareParameter(source + ".class_types",
                     rclcpp::ParameterValue(std::vector<std::string>({})));
    declareParameter(source + ".max_obstacle_distance", rclcpp::ParameterValue(5.0));
    declareParameter(source + ".min_obstacle_distance", rclcpp::ParameterValue(0.3));
    declareParameter(source + ".tile_map_decay_time",   rclcpp::ParameterValue(5.0));
    declareParameter(source + ".visualize_tile_map",    rclcpp::ParameterValue(false));
    declareParameter(source + ".use_cost_selection",    rclcpp::ParameterValue(true));

    node->get_parameter(name_ + "." + source + ".segmentation_topic",   segmentation_topic);
    node->get_parameter(name_ + "." + source + ".confidence_topic",     confidence_topic);
    node->get_parameter(name_ + "." + source + ".labels_topic",         labels_topic);
    node->get_parameter(name_ + "." + source + ".pointcloud_topic",     pointcloud_topic);
    node->get_parameter(name_ + "." + source + ".observation_persistence", observation_keep_time);
    node->get_parameter(name_ + "." + source + ".expected_update_rate", expected_update_rate);
    node->get_parameter(name_ + "." + source + ".class_types",          class_types_string);
    node->get_parameter(name_ + "." + source + ".max_obstacle_distance", max_obstacle_distance);
    node->get_parameter(name_ + "." + source + ".min_obstacle_distance", min_obstacle_distance);
    node->get_parameter(name_ + "." + source + ".tile_map_decay_time",  tile_map_decay_time);
    node->get_parameter(name_ + "." + source + ".visualize_tile_map",   visualize_tile_map);
    bool use_cost_selection = true;
    node->get_parameter(name_ + "." + source + ".use_cost_selection",   use_cost_selection);

    if (class_types_string.empty()) {
      RCLCPP_ERROR(logger_,
        "No class types defined for source %s — plugin cannot work.", source.c_str());
      exit(-1);
    }

    std::unordered_map<std::string, CostHeuristicParams>      class_map;
    std::unordered_map<std::string, std::vector<std::string>> class_type_to_names;

    for (auto& class_type : class_types_string) {
      std::vector<std::string> classes_ids;
      declareParameter(source + "." + class_type + ".classes",
                       rclcpp::ParameterValue(std::vector<std::string>({})));
      declareParameter(source + "." + class_type + ".base_cost",          rclcpp::ParameterValue(0));
      declareParameter(source + "." + class_type + ".max_cost",           rclcpp::ParameterValue(0));
      declareParameter(source + "." + class_type + ".mark_confidence",    rclcpp::ParameterValue(0));
      declareParameter(source + "." + class_type + ".samples_to_max_cost",rclcpp::ParameterValue(0));
      declareParameter(source + "." + class_type + ".dominant_priority",  rclcpp::ParameterValue(false));

      node->get_parameter(name_ + "." + source + "." + class_type + ".classes", classes_ids);
      if (classes_ids.empty()) {
        RCLCPP_ERROR(logger_, "No classes defined on type %s", class_type.c_str());
        continue;
      }
      class_type_to_names[class_type] = classes_ids;

      CostHeuristicParams cost_params;
      node->get_parameter(name_ + "." + source + "." + class_type + ".base_cost",
                          cost_params.base_cost);
      node->get_parameter(name_ + "." + source + "." + class_type + ".max_cost",
                          cost_params.max_cost);
      node->get_parameter(name_ + "." + source + "." + class_type + ".mark_confidence",
                          cost_params.mark_confidence);
      node->get_parameter(name_ + "." + source + "." + class_type + ".samples_to_max_cost",
                          cost_params.samples_to_max_cost);
      node->get_parameter(name_ + "." + source + "." + class_type + ".dominant_priority",
                          cost_params.dominant_priority);

      for (auto& class_id : classes_ids)
        class_map.emplace(class_id, cost_params);
    }

    if (class_map.empty()) {
      RCLCPP_ERROR(logger_,
        "No classes defined for source %s — plugin cannot work.", source.c_str());
      exit(-1);
    }

    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_;
    rmw_qos_profile_t custom_qos = rmw_qos_profile_sensor_data;
    custom_qos.depth = 50;

    rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> tl_sub_opt;
    tl_sub_opt.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    tl_sub_opt.callback_group         = callback_group_;
    rmw_qos_profile_t tl_qos  = rmw_qos_profile_default;
    tl_qos.depth               = 5;
    tl_qos.reliability         = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    tl_qos.durability          = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

    auto segmentation_buffer = std::make_shared<nav2_costmap_2d::SegmentationBuffer>(
      node, source, class_types_string, class_map, class_type_to_names,
      observation_keep_time, expected_update_rate,
      max_obstacle_distance, min_obstacle_distance,
      *tf_, global_frame_, "",
      tf2::durationFromSec(transform_tolerance),
      getResolution(), tile_map_decay_time,
      visualize_tile_map, use_cost_selection);

    segmentation_buffers_.push_back(segmentation_buffer);

    auto seg_sub = std::make_shared<ImageSub>(
      node, segmentation_topic, custom_qos, sub_opt);
    seg_sub->unsubscribe();
    semantic_segmentation_subs_.push_back(seg_sub);

    auto label_sub = std::make_shared<LabelSub>(
      node, labels_topic, tl_qos, tl_sub_opt);
    label_sub->registerCallback(
      std::bind(&SemanticSegmentationLayer::labelinfoCb, this,
                std::placeholders::_1, segmentation_buffers_.back()));
    label_sub->unsubscribe();
    label_info_subs_.push_back(label_sub);

    auto pc_sub = std::make_shared<PC2Sub>(
      node, pointcloud_topic, custom_qos, sub_opt);
    pc_sub->unsubscribe();
    pointcloud_subs_.push_back(pc_sub);

    auto pc_tf_sub = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
      *pointcloud_subs_.back(), *tf_, global_frame_, 50,
      node->get_node_logging_interface(),
      node->get_node_clock_interface(),
      tf2::durationFromSec(transform_tolerance));
    pointcloud_tf_subs_.push_back(pc_tf_sub);

    if (!confidence_topic.empty()) {
      auto conf_sub = std::make_shared<ImageSub>(
        node, confidence_topic, custom_qos, sub_opt);
      conf_sub->unsubscribe();
      semantic_segmentation_confidence_subs_.push_back(conf_sub);

      auto sync = std::make_shared<SyncSegmConfPC>(1000);
      sync->connectInput(*semantic_segmentation_subs_.back(),
                         *semantic_segmentation_confidence_subs_.back(),
                         *pointcloud_tf_subs_.back());
      sync->registerCallback(
        std::bind(&SemanticSegmentationLayer::syncSegmConfPointcloudCb, this,
                  std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                  segmentation_buffers_.back()));
      segm_conf_pc_notifiers_.push_back(sync);
      RCLCPP_INFO(logger_, "Confidence enabled for source %s", source.c_str());
    } else {
      RCLCPP_WARN(logger_,
        "Confidence topic empty for source %s — confidence not used.", source.c_str());
      auto sync = std::make_shared<SyncSegmPC>(1000);
      sync->connectInput(*semantic_segmentation_subs_.back(),
                         *pointcloud_tf_subs_.back());
      sync->registerCallback(
        std::bind(&SemanticSegmentationLayer::syncSegmPointcloudCb, this,
                  std::placeholders::_1, std::placeholders::_2,
                  segmentation_buffers_.back()));
      segm_pc_notifiers_.push_back(sync);
    }
  }

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&SemanticSegmentationLayer::dynamicParametersCallback, this,
              std::placeholders::_1));
}

void SemanticSegmentationLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
  double* min_x, double* min_y, double* max_x, double* max_y)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  if (rolling_window_)
    updateOrigin(robot_x - getSizeInMetersX() / 2,
                 robot_y - getSizeInMetersY() / 2);

  if (!enabled_) return;

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

  // Snapshot map dimensions once after updateOrigin() so that worldToMap()
  // and index arithmetic use a consistent size_x_ / size_y_.
  const unsigned int map_size_x = getSizeInCellsX();
  const unsigned int map_size_y = getSizeInCellsY();
  const unsigned int map_size   = map_size_x * map_size_y;

  for (auto& buffer : segmentation_buffers_) {
    // Hold buffer_lock for the ENTIRE iteration.
    // getCostForClassId() reads segmentation_cost_multimap_ which
    // dynamicParametersCallback() writes via updateClassMap().
    // Releasing early would create a data race.
    std::unique_lock<std::recursive_mutex> buffer_lock(buffer->getMutex());

    SegmentationTileMap::SharedPtr tile_map = buffer->getSegmentationTileMap();
    if (!tile_map) continue;

    tile_map->purgeOldObservations(current_time);

    // Value-copy snapshot — owns all data independently of tile_map's
    // internal container, safe against concurrent rehash/reallocation.
    struct TileSnapshot {
      TileIndex                index;
      TileWorldXY              world_coords;
      TemporalObservationQueue obs_queue;  // full value copy
    };

    std::vector<TileSnapshot> tile_snapshot;
    tile_snapshot.reserve(tile_map->size());

    for (auto& tile : *tile_map) {
      if (!tile.second.empty()) {
        tile_snapshot.push_back({
          tile.first,
          tile_map->indexToWorld(tile.first.x, tile.first.y),
          tile.second
        });
      }
    }

    for (auto& snap : tile_snapshot) {
      unsigned int mx, my;
      if (!worldToMap(snap.world_coords.x, snap.world_coords.y, mx, my)) {
        RCLCPP_DEBUG(logger_, "Computing map coords failed");
        continue;
      }

      if (mx >= map_size_x || my >= map_size_y) {
        RCLCPP_WARN(logger_,
          "Tile (%u,%u) out of map bounds (%u x %u) — skipping",
          mx, my, map_size_x, map_size_y);
        continue;
      }

      unsigned int index = mx + my * map_size_x;
      if (index >= map_size) {
        RCLCPP_ERROR(logger_,
          "Computed index %u exceeds map size %u — skipping", index, map_size);
        continue;
      }

      CostHeuristicParams cost_params =
        buffer->getCostForClassId(snap.obs_queue.getClassId());

      costmap_[index] =
        (static_cast<int>(snap.obs_queue.size()) >= cost_params.samples_to_max_cost &&
         snap.obs_queue.getConfidenceSum() / snap.obs_queue.size() >
           cost_params.mark_confidence)
        ? cost_params.max_cost
        : cost_params.base_cost;

      touch(snap.world_coords.x, snap.world_coords.y, min_x, min_y, max_x, max_y);
    }
    // buffer_lock released by unique_lock destructor
  }

  current_ = true;
}

void SemanticSegmentationLayer::onFootprintChanged()
{
  RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"),
    "SemanticSegmentationLayer::onFootprintChanged(): %lu footprint points",
    layered_costmap_->getFootprint().size());
}

void SemanticSegmentationLayer::updateCosts(
  nav2_costmap_2d::Costmap2D& master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) return;

  if (!current_ && was_reset_) {
    was_reset_ = false;
    current_   = true;
  }
  if (!costmap_) return;

  switch (combination_method_) {
    case 0: updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j); break;
    case 1: updateWithMax    (master_grid, min_i, min_j, max_i, max_j); break;
    default: break;
  }
}

// FIX: buffer_lock acquired BEFORE calling createSegmentationCostMultimap
// so that the shared_ptr replacement is never concurrent with bufferSegmentation.
void SemanticSegmentationLayer::labelinfoCb(
  const std::shared_ptr<const vision_msgs::msg::LabelInfo>& label_info,
  const std::shared_ptr<nav2_costmap_2d::SegmentationBuffer>& buffer)
{
  std::unique_lock<std::recursive_mutex> buffer_lock(buffer->getMutex());
  buffer->createSegmentationCostMultimap(*label_info);
}

void SemanticSegmentationLayer::syncSegmPointcloudCb(
  const std::shared_ptr<const sensor_msgs::msg::Image>&       segmentation,
  const std::shared_ptr<const sensor_msgs::msg::PointCloud2>& pointcloud,
  const std::shared_ptr<nav2_costmap_2d::SegmentationBuffer>& buffer)
{
  if (segmentation->width * segmentation->height !=
      pointcloud->width * pointcloud->height) {
    RCLCPP_WARN(logger_,
      "Pointcloud/segmentation size mismatch — skipping. "
      "seg=(%u x %u) pc=(%u x %u)",
      segmentation->width, segmentation->height,
      pointcloud->width,   pointcloud->height);
    return;
  }

  unsigned expected = segmentation->width * segmentation->height;
  if (segmentation->data.size() < expected) {
    RCLCPP_WARN(logger_,
      "Segmentation data too small: got %zu, expected %u — skipping",
      segmentation->data.size(), expected);
    return;
  }

  // FIX: acquire lock BEFORE isClassIdCostMapEmpty() so the check and the
  // subsequent bufferSegmentation() are atomic with respect to labelinfoCb.
  std::unique_lock<std::recursive_mutex> buffer_lock(buffer->getMutex());

  if (buffer->isClassIdCostMapEmpty()) {
    RCLCPP_WARN(logger_,
      "Class map empty (no LabelInfo received yet) for source %s — skipping",
      buffer->getBufferSource().c_str());
    return;
  }

  sensor_msgs::msg::Image conf_mask = *segmentation;
  std::fill(conf_mask.data.begin(), conf_mask.data.end(), 255);
  buffer->bufferSegmentation(*pointcloud, *segmentation, conf_mask);
}

void SemanticSegmentationLayer::syncSegmConfPointcloudCb(
  const std::shared_ptr<const sensor_msgs::msg::Image>&       segmentation,
  const std::shared_ptr<const sensor_msgs::msg::Image>&       confidence,
  const std::shared_ptr<const sensor_msgs::msg::PointCloud2>& pointcloud,
  const std::shared_ptr<nav2_costmap_2d::SegmentationBuffer>& buffer)
{
  if (segmentation->width * segmentation->height !=
      pointcloud->width * pointcloud->height) {
    RCLCPP_WARN(logger_,
      "Pointcloud/segmentation size mismatch — skipping. "
      "seg=(%u x %u) pc=(%u x %u)",
      segmentation->width, segmentation->height,
      pointcloud->width,   pointcloud->height);
    return;
  }

  unsigned expected = segmentation->width * segmentation->height;
  if (segmentation->data.size() < expected) {
    RCLCPP_WARN(logger_,
      "Segmentation data too small: got %zu, expected %u — skipping",
      segmentation->data.size(), expected);
    return;
  }

  // FIX: acquire lock BEFORE isClassIdCostMapEmpty() — same rationale as above.
  std::unique_lock<std::recursive_mutex> buffer_lock(buffer->getMutex());

  if (buffer->isClassIdCostMapEmpty()) {
    RCLCPP_WARN(logger_,
      "Class map empty (no LabelInfo received yet) for source %s — skipping",
      buffer->getBufferSource().c_str());
    return;
  }

  buffer->bufferSegmentation(*pointcloud, *segmentation, *confidence);
}

void SemanticSegmentationLayer::reset()
{
  resetMaps();
  current_   = false;
  was_reset_ = true;
}

bool SemanticSegmentationLayer::getSegmentationTileMaps(
  std::vector<std::pair<SegmentationTileMap::SharedPtr,
                        SegmentationBuffer::SharedPtr>>& segmentation_tile_maps)
{
  for (auto& buf : segmentation_buffers_) {
    std::unique_lock<std::recursive_mutex> buffer_lock(buf->getMutex());
    segmentation_tile_maps.emplace_back(buf->getSegmentationTileMap(), buf);
    // WARNING: lock released here. Callers must hold buf->getMutex() themselves
    // for the entire duration that the returned tile_map is accessed.
  }
  return true;
}

rcl_interfaces::msg::SetParametersResult
SemanticSegmentationLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  auto result = rcl_interfaces::msg::SetParametersResult();

  for (auto& parameter : parameters) {
    const auto& type = parameter.get_type();
    const auto& name = parameter.get_name();

    if (type == rclcpp::ParameterType::PARAMETER_BOOL) {
      if (name == name_ + ".enabled")
        enabled_ = parameter.as_bool();
    }

    std::stringstream ss(topics_string_);
    std::string source;
    while (ss >> source) {
      if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        for (auto& buffer : segmentation_buffers_) {
          if (buffer->getBufferSource() != source) continue;
          if (name == name_ + "." + source + ".max_obstacle_distance")
            buffer->setMaxObstacleDistance(parameter.as_double());
          else if (name == name_ + "." + source + ".min_obstacle_distance")
            buffer->setMinObstacleDistance(parameter.as_double());
        }
      } else if (type == rclcpp::ParameterType::PARAMETER_INTEGER) {
        for (auto& buffer : segmentation_buffers_) {
          if (buffer->getBufferSource() != source) continue;

          // FIX: hold buffer->getMutex() around the read-modify-write cycle
          // on cost params to prevent a TOCTOU race with getCostForClassName
          // followed by updateClassMap on the same entry.
          std::unique_lock<std::recursive_mutex> buf_lock(buffer->getMutex());

          for (auto& class_type : buffer->getClassTypes()) {
            auto class_names = buffer->getClassNamesForType(class_type);
            for (auto& class_name : class_names) {
              CostHeuristicParams p = buffer->getCostForClassName(class_name);

              if (name == name_ + "." + source + "." + class_type + ".base_cost")
                p.base_cost = static_cast<uint8_t>(parameter.as_int());
              else if (name == name_ + "." + source + "." + class_type + ".max_cost")
                p.max_cost = static_cast<uint8_t>(parameter.as_int());
              else if (name == name_ + "." + source + "." + class_type + ".mark_confidence")
                p.mark_confidence = static_cast<uint8_t>(parameter.as_int());
              else if (name == name_ + "." + source + "." + class_type + ".samples_to_max_cost")
                p.samples_to_max_cost = parameter.as_int();
              else if (name == name_ + "." + source + "." + class_type + ".dominant_priority")
                p.dominant_priority = parameter.as_bool();
              else
                continue;

              buffer->updateClassMap(class_name, p);
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
  for (auto& s : semantic_segmentation_subs_)            if (s) s->subscribe();
  for (auto& s : semantic_segmentation_confidence_subs_) if (s) s->subscribe();
  for (auto& s : label_info_subs_)                       if (s) s->subscribe();
  for (auto& s : pointcloud_subs_)                       if (s) s->subscribe();
}

void SemanticSegmentationLayer::deactivate()
{
  for (auto& s : semantic_segmentation_subs_)            if (s) s->unsubscribe();
  for (auto& s : semantic_segmentation_confidence_subs_) if (s) s->unsubscribe();
  for (auto& s : label_info_subs_)                       if (s) s->unsubscribe();
  for (auto& s : pointcloud_subs_)                       if (s) s->unsubscribe();
}

}  // namespace nav2_costmap_2d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::SemanticSegmentationLayer,
                       nav2_costmap_2d::Layer)
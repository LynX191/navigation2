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
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Pedro Gonzalez (pedro@robot.com)
 *          Johan Solarte (jsolarte@robot.com)
 *********************************************************************/
#include "nav2_costmap_2d/segmentation_buffer.hpp"

#include <algorithm>
#include <chrono>
#include <list>
#include <string>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/convert.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace nav2_costmap_2d {

SegmentationBuffer::SegmentationBuffer(
  const nav2_util::LifecycleNode::WeakPtr&                 parent,
  std::string                                              buffer_source,
  std::vector<std::string>                                 class_types,
  std::unordered_map<std::string, CostHeuristicParams>     class_names_cost_map,
  std::unordered_map<std::string, std::vector<std::string>> class_type_to_names,
  double observation_keep_time,
  double expected_update_rate,
  double max_lookahead_distance,
  double min_lookahead_distance,
  tf2_ros::Buffer& tf2_buffer,
  std::string      global_frame,
  std::string      sensor_frame,
  tf2::Duration    tf_tolerance,
  double           costmap_resolution,
  double           tile_map_decay_time,
  bool             visualize_tile_map,
  bool             use_cost_selection)
: tf2_buffer_(tf2_buffer)
, class_types_(class_types)
, class_names_cost_map_(class_names_cost_map)
, class_type_to_names_(class_type_to_names)
, observation_keep_time_(rclcpp::Duration::from_seconds(observation_keep_time))
, expected_update_rate_(rclcpp::Duration::from_seconds(expected_update_rate))
, global_frame_(global_frame)
, sensor_frame_(sensor_frame)
, buffer_source_(buffer_source)
, sq_max_lookahead_distance_(std::pow(max_lookahead_distance, 2))
, sq_min_lookahead_distance_(std::pow(min_lookahead_distance, 2))
, tf_tolerance_(tf_tolerance)
{
  auto node = parent.lock();
  clock_           = node->get_clock();
  logger_          = node->get_logger();
  last_updated_    = node->now();
  temporal_tile_map_ =
    std::make_shared<SegmentationTileMap>(costmap_resolution, tile_map_decay_time);
  visualize_tile_map_ = visualize_tile_map;
  use_cost_selection_ = use_cost_selection;

  // Initialise with an empty multimap so isClassIdCostMapEmpty() is always safe
  // to call before the first LabelInfo message arrives.
  segmentation_cost_multimap_ = std::make_shared<SegmentationCostMultimap>();

  RCLCPP_INFO(logger_,
    "SegmentationBuffer [%s]: Selection method = %s",
    buffer_source_.c_str(),
    use_cost_selection_ ? "COST-BASED (max_cost)" : "CONFIDENCE-BASED");

  if (visualize_tile_map_) {
    tile_map_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      buffer_source + "/tile_map", 1);
  }
}

SegmentationBuffer::~SegmentationBuffer() {}

// MUST be called while holding getMutex().
void SegmentationBuffer::createSegmentationCostMultimap(
  const vision_msgs::msg::LabelInfo& label_info)
{
  std::unordered_map<std::string, uint8_t> class_to_id_map;
  for (const auto& semantic_class : label_info.class_map) {
    const auto& name = semantic_class.class_name;
    if (class_names_cost_map_.find(name) == class_names_cost_map_.end()) {
      RCLCPP_ERROR(logger_,
        "CRITICAL ERROR: Class '%s' from label_info is not in costmap parameters. "
        "It will be ignored.", name.c_str());
      continue;
    }
    class_to_id_map[name] = semantic_class.class_id;
  }
  // Atomically replace the shared_ptr under the caller's buffer lock.
  segmentation_cost_multimap_ =
    std::make_shared<SegmentationCostMultimap>(class_to_id_map, class_names_cost_map_);
}

// MUST be called while holding getMutex().
void SegmentationBuffer::bufferSegmentation(
  const sensor_msgs::msg::PointCloud2& cloud,
  const sensor_msgs::msg::Image&        segmentation,
  const sensor_msgs::msg::Image&        confidence)
{
  geometry_msgs::msg::PointStamped global_origin;
  std::string origin_frame =
    sensor_frame_.empty() ? cloud.header.frame_id : sensor_frame_;

  try {
    geometry_msgs::msg::PointStamped local_origin;
    local_origin.header.stamp     = cloud.header.stamp;
    local_origin.header.frame_id  = origin_frame;
    local_origin.point.x = local_origin.point.y = local_origin.point.z = 0;
    tf2_buffer_.transform(local_origin, global_origin, global_frame_, tf_tolerance_);

    sensor_msgs::msg::PointCloud2 global_frame_cloud;
    tf2_buffer_.transform(cloud, global_frame_cloud, global_frame_, tf_tolerance_);
    global_frame_cloud.header.stamp = cloud.header.stamp;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(global_frame_cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(global_frame_cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(global_frame_cloud, "z");

    std::unordered_map<TileIndex, int> best_observations_idxs;
    double cloud_time_seconds =
      rclcpp::Time(cloud.header.stamp.sec, cloud.header.stamp.nanosec).seconds();

    for (size_t v = 0; v < segmentation.height; ++v) {
      for (size_t u = 0; u < segmentation.width; ++u) {
        int pixel_idx = static_cast<int>(v * segmentation.width + u);

        if (!std::isfinite(*iter_z)) {
          ++iter_x; ++iter_y; ++iter_z;
          continue;
        }

        double sq_dist =
          std::pow(*iter_x - global_origin.point.x, 2) +
          std::pow(*iter_y - global_origin.point.y, 2) +
          std::pow(*iter_z - global_origin.point.z, 2);

        if (sq_dist >= sq_max_lookahead_distance_ ||
            sq_dist <= sq_min_lookahead_distance_) {
          ++iter_x; ++iter_y; ++iter_z;
          continue;
        }

        TileIndex costmap_index =
          temporal_tile_map_->worldToIndex(*iter_x, *iter_y);

        auto it = best_observations_idxs.find(costmap_index);
        if (it != best_observations_idxs.end()) {
          if (use_cost_selection_) {
            uint8_t cur_class = segmentation.data[pixel_idx];
            uint8_t ext_class = segmentation.data[it->second];
            auto cur_cost = segmentation_cost_multimap_->getCostById(cur_class);
            auto ext_cost = segmentation_cost_multimap_->getCostById(ext_class);
            if (cur_cost.max_cost > ext_cost.max_cost) {
              best_observations_idxs[costmap_index] = pixel_idx;
              RCLCPP_DEBUG(logger_,
                "COST-BASED: Replaced tile obs — cur=%d (max_cost=%d) > ext=%d (max_cost=%d)",
                cur_class, cur_cost.max_cost, ext_class, ext_cost.max_cost);
            }
          } else {
            if (confidence.data[pixel_idx] > confidence.data[it->second]) {
              best_observations_idxs[costmap_index] = pixel_idx;
              RCLCPP_DEBUG(logger_,
                "CONFIDENCE-BASED: Replaced tile obs — cur=%d > ext=%d",
                confidence.data[pixel_idx], confidence.data[it->second]);
            }
          }
        } else {
          best_observations_idxs[costmap_index] = pixel_idx;
        }

        ++iter_x; ++iter_y; ++iter_z;
      }
    }

    // Commit best observations into the tile map under the tile map's own mutex.
    std::unique_lock<std::recursive_mutex> tile_lock(temporal_tile_map_->getMutex());
    temporal_tile_map_->purgeOldObservations(cloud_time_seconds);

    for (auto& idx : best_observations_idxs) {
      int       img_idx      = idx.second;
      TileIndex tile_index   = idx.first;
      uint8_t   class_id     = segmentation.data[img_idx];

      if (segmentation_cost_multimap_->hasClassId(class_id)) {
        TileObservation best_obs{
          class_id,
          static_cast<float>(confidence.data[img_idx]),
          cloud_time_seconds
        };
        bool dominant_priority =
          segmentation_cost_multimap_->getCostById(class_id).dominant_priority;
        temporal_tile_map_->pushObservation(best_obs, tile_index, dominant_priority);
      } else {
        RCLCPP_DEBUG(logger_,
          "SegmentationBuffer [%s]: Skipping undefined class_id %d in tile (%d,%d)",
          buffer_source_.c_str(), class_id, tile_index.x, tile_index.y);
      }
    }

    if (visualize_tile_map_) {
      tile_map_pub_->publish(visualizeTemporalTileMap(*temporal_tile_map_));
    }

  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(logger_,
      "TF Exception: sensor_frame=%s, cloud_frame=%s, what=%s",
      sensor_frame_.c_str(), cloud.header.frame_id.c_str(), ex.what());
    return;
  }

  last_updated_ = clock_->now();
}

std::unordered_map<std::string, CostHeuristicParams> SegmentationBuffer::getClassMap()
{
  return class_names_cost_map_;
}

std::vector<std::string> SegmentationBuffer::getClassNamesForType(
  const std::string& class_type)
{
  auto it = class_type_to_names_.find(class_type);
  return (it != class_type_to_names_.end()) ? it->second : std::vector<std::string>();
}

// MUST be called while holding getMutex().
void SegmentationBuffer::updateClassMap(std::string new_class,
                                        CostHeuristicParams new_cost)
{
  segmentation_cost_multimap_->updateCostByName(new_class, new_cost);
}

bool SegmentationBuffer::isCurrent() const
{
  if (expected_update_rate_ == rclcpp::Duration(0.0s))
    return true;

  bool current = (clock_->now() - last_updated_) <= expected_update_rate_;
  if (!current) {
    RCLCPP_WARN(logger_,
      "The %s segmentation buffer has not been updated for %.2f seconds "
      "(expected every %.2f seconds).",
      buffer_source_.c_str(),
      (clock_->now() - last_updated_).seconds(),
      expected_update_rate_.seconds());
  }
  return current;
}

void SegmentationBuffer::resetLastUpdated() { last_updated_ = clock_->now(); }

}  // namespace nav2_costmap_2d
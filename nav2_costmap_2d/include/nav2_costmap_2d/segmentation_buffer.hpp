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
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Pedro Gonzalez (pedro@robot.com)
 *          Johan Solarte (jsolarte@robot.com)
 *********************************************************************/

#ifndef SEMANTIC_SEGMENTATION_LAYER__SEGMENTATION_BUFFER_HPP_
#define SEMANTIC_SEGMENTATION_LAYER__SEGMENTATION_BUFFER_HPP_

#include <deque>
#include <list>
#include <string>
#include <vector>

#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "vision_msgs/msg/label_info.hpp"

/**
 * @brief Represents the parameters associated with the cost calculation
 *        for a given class.
 */
struct CostHeuristicParams
{
  uint8_t base_cost, max_cost, mark_confidence;
  int     samples_to_max_cost;
  bool    dominant_priority;
};

/**
 * @brief Represents a 2D grid index with equality comparison.
 *        Supports negative indexes.
 */
struct TileIndex {
  int x, y;
  bool operator==(const TileIndex& other) const {
    return x == other.x && y == other.y;
  }
};

namespace std {
/**
 * @brief Custom hash for TileIndex to enable use as unordered_map key.
 */
template<>
struct hash<TileIndex> {
  size_t operator()(const TileIndex& coord) const {
    return std::hash<int>()(coord.x) ^ (std::hash<int>()(coord.y) << 1);
  }
};
}  // namespace std

/**
 * @brief Represents the world coordinates of a tile.
 */
struct TileWorldXY { double x, y; };

/**
 * @brief Encapsulates a single observation for a tile.
 */
struct TileObservation {
  using UniquePtr = std::unique_ptr<TileObservation>;
  uint8_t class_id;
  float   confidence;
  double  timestamp;
};

/**
 * @brief Manages temporal observations with a decay mechanism.
 *
 * Wraps per-class deques and tracks the dominant class (most samples).
 * Sentinel value -1 means no dominant class exists.
 */
class TemporalObservationQueue
{
public:
  TemporalObservationQueue() {}

  /**
   * @brief Add an observation; updates dominant-class bookkeeping.
   * @param tile_obs          The observation to add.
   * @param dominant_priority When true this class immediately becomes dominant.
   */
  void push(TileObservation tile_obs, bool dominant_priority = false)
  {
    uint8_t class_id = tile_obs.class_id;
    auto& queue = class_queues_[class_id];
    queue.push_back(tile_obs);
    class_confidence_sums_[class_id] += tile_obs.confidence;

    size_t current_class_size = queue.size();
    bool should_become_dominant =
      dominant_priority || (current_class_size > dominant_class_size_);

    if (should_become_dominant) {
      if (dominant_class_id_ != -1 && dominant_class_id_ != class_id) {
        clearQueuesExcept(class_id);
      }
      setDominant(class_id, current_class_size);
    }
  }

  bool   empty() const { return dominant_class_id_ == -1; }
  size_t size()  const { return dominant_class_size_; }

  void setDecayTime(float decay_time) { decay_time_ = decay_time; }

  float getConfidenceSum() const
  {
    if (dominant_class_id_ != -1) {
      auto it = class_confidence_sums_.find(dominant_class_id_);
      return (it != class_confidence_sums_.end()) ? it->second : 0.0f;
    }
    return 0.0f;
  }

  int getClassId() const { return dominant_class_id_; }

  std::deque<TileObservation> getQueue()
  {
    if (dominant_class_id_ != -1) {
      auto it = class_queues_.find(dominant_class_id_);
      return (it != class_queues_.end())
               ? it->second
               : std::deque<TileObservation>();
    }
    return std::deque<TileObservation>();
  }

  /**
   * @brief Remove observations older than decay_time_ from all queues.
   * @param current_time Current time in seconds.
   */
  void purgeOld(double current_time)
  {
    bool dominant_removed = false;
    for (auto it = class_queues_.begin(); it != class_queues_.end(); ) {
      auto&         queue    = it->second;
      const uint8_t class_id = it->first;

      while (!queue.empty()) {
        if (current_time - queue.front().timestamp > decay_time_) {
          class_confidence_sums_[class_id] -= queue.front().confidence;
          queue.pop_front();
        } else {
          break;
        }
      }

      if (queue.empty()) {
        if (class_id == static_cast<uint8_t>(dominant_class_id_))
          dominant_removed = true;
        class_confidence_sums_.erase(class_id);
        it = class_queues_.erase(it);
      } else {
        ++it;
      }
    }

    if (dominant_removed) {
      recomputeDominant();
    } else if (dominant_class_id_ != -1) {
      auto it = class_queues_.find(dominant_class_id_);
      if (it != class_queues_.end())
        setDominant(dominant_class_id_, it->second.size());
      else
        resetDominant();
    }
  }

private:
  void clearQueuesExcept(uint8_t keep_class_id)
  {
    for (auto it = class_queues_.begin(); it != class_queues_.end(); ) {
      if (it->first != keep_class_id) {
        class_confidence_sums_.erase(it->first);
        it = class_queues_.erase(it);
      } else {
        ++it;
      }
    }
  }

  void recomputeDominant()
  {
    resetDominant();
    for (const auto& pair : class_queues_) {
      if (pair.second.size() > dominant_class_size_)
        setDominant(pair.first, pair.second.size());
    }
  }

  void resetDominant() { dominant_class_id_ = -1; dominant_class_size_ = 0; }

  void setDominant(uint8_t class_id, size_t size)
  {
    dominant_class_id_   = class_id;
    dominant_class_size_ = size;
  }

  std::unordered_map<uint8_t, std::deque<TileObservation>> class_queues_;
  std::unordered_map<uint8_t, float>                        class_confidence_sums_;
  int    dominant_class_id_   = -1;
  size_t dominant_class_size_ = 0;
  double decay_time_          = 0.0;
};

/**
 * @brief Manages a map of tile observations indexed by TileIndex.
 *        Has its own recursive_mutex for thread safety.
 */
class SegmentationTileMap
{
public:
  using SharedPtr     = std::shared_ptr<SegmentationTileMap>;
  using Iterator      = std::unordered_map<TileIndex, TemporalObservationQueue>::iterator;
  using ConstIterator = std::unordered_map<TileIndex, TemporalObservationQueue>::const_iterator;

  SegmentationTileMap(float resolution, float decay_time)
  : resolution_(resolution), decay_time_(decay_time)
  {
    tile_map_.reserve(1e4);
  }
  SegmentationTileMap() {}

  Iterator      begin()       { return tile_map_.begin(); }
  ConstIterator begin() const { return tile_map_.begin(); }
  Iterator      end()         { return tile_map_.end(); }
  ConstIterator end()   const { return tile_map_.end(); }

  inline void lock()   { lock_.lock(); }
  inline void unlock() { lock_.unlock(); }
  inline std::recursive_mutex& getMutex() { return lock_; }

  int size() { return static_cast<int>(tile_map_.size()); }

  TileIndex worldToIndex(double x, double y) const
  {
    return TileIndex{
      static_cast<int>(std::floor(x / resolution_)),
      static_cast<int>(std::floor(y / resolution_))
    };
  }

  TileWorldXY indexToWorld(int x, int y) const
  {
    return TileWorldXY{
      (static_cast<double>(x) + 0.5) * resolution_,
      (static_cast<double>(y) + 0.5) * resolution_
    };
  }

  void pushObservation(TileObservation& obs, TileIndex& idx,
                       bool dominant_priority = false)
  {
    auto it = tile_map_.find(idx);
    if (it != tile_map_.end()) {
      it->second.push(obs, dominant_priority);
    } else {
      TemporalObservationQueue& queue = tile_map_[idx];
      queue.setDecayTime(decay_time_);
      queue.push(obs, dominant_priority);
    }
  }

  void purgeOldObservations(double current_time)
  {
    std::vector<TileIndex> tiles_to_remove;
    for (auto& tile : tile_map_) {
      tile.second.purgeOld(current_time);
      if (tile.second.empty())
        tiles_to_remove.emplace_back(tile.first);
    }
    for (auto& tile : tiles_to_remove)
      tile_map_.erase(tile);
  }

private:
  std::unordered_map<TileIndex, TemporalObservationQueue> tile_map_;
  float              resolution_  = 0.0f;
  float              decay_time_  = 0.0f;
  std::recursive_mutex lock_;
};

/**
 * @brief POD type used to snapshot per-tile data in updateBounds().
 */
struct PointData {
  float   x, y, z;
  float   confidence, confidence_sum;
  uint8_t class_id;
};

inline sensor_msgs::msg::PointCloud2
visualizeTemporalTileMap(SegmentationTileMap& tileMap)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "map";
  cloud.header.stamp    = rclcpp::Clock().now();

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    6,
    "x",              1, sensor_msgs::msg::PointField::FLOAT32,
    "y",              1, sensor_msgs::msg::PointField::FLOAT32,
    "z",              1, sensor_msgs::msg::PointField::FLOAT32,
    "confidence",     1, sensor_msgs::msg::PointField::FLOAT32,
    "confidence_sum", 1, sensor_msgs::msg::PointField::FLOAT32,
    "class",          1, sensor_msgs::msg::PointField::UINT8);

  std::vector<PointData> points;
  for (auto& tile : tileMap) {
    TileIndex   idx     = tile.first;
    TileWorldXY worldXY = tileMap.indexToWorld(idx.x, idx.y);
    double      z       = 0.0;
    for (auto& obs : tile.second.getQueue()) {
      PointData point;
      point.x              = worldXY.x;
      point.y              = worldXY.y;
      point.z              = z;
      point.confidence     = obs.confidence;
      point.confidence_sum = (tile.second.size() > 0)
                               ? tile.second.getConfidenceSum() / tile.second.size()
                               : 0.0f;
      point.class_id       = static_cast<uint8_t>(obs.class_id);
      points.push_back(point);
      z += 0.02;
    }
  }

  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float>   iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float>   iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float>   iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float>   iter_conf(cloud, "confidence");
  sensor_msgs::PointCloud2Iterator<float>   iter_conf_sum(cloud, "confidence_sum");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_class(cloud, "class");

  for (const auto& p : points) {
    *iter_x        = p.x;
    *iter_y        = p.y;
    *iter_z        = p.z;
    *iter_conf     = p.confidence;
    *iter_conf_sum = p.confidence_sum;
    *iter_class    = p.class_id;
    ++iter_x; ++iter_y; ++iter_z; ++iter_conf; ++iter_conf_sum; ++iter_class;
  }
  return cloud;
}

/**
 * @brief Maps class names↔IDs and manages CostHeuristicParams per class.
 *
 * All public methods are individually thread-safe via an internal mutex.
 * The shared_ptr to this object must only be replaced while the owning
 * SegmentationBuffer's recursive_mutex is held (see createSegmentationCostMultimap).
 */
class SegmentationCostMultimap
{
public:
  using SharedPtr = std::shared_ptr<SegmentationCostMultimap>;

  SegmentationCostMultimap() {}

  SegmentationCostMultimap(
    const std::unordered_map<std::string, uint8_t>&             nameToIdMap,
    const std::unordered_map<std::string, CostHeuristicParams>& nameToCostMap)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    name_to_id_ = nameToIdMap;
    for (const auto& pair : nameToIdMap) {
      uint8_t    id      = pair.second;
      auto       cost_it = nameToCostMap.find(pair.first);
      id_to_cost_[id]    = (cost_it != nameToCostMap.end())
                             ? cost_it->second
                             : CostHeuristicParams{0, 0, 0, 0, false};
    }
  }

  void updateCostById(uint8_t id, const CostHeuristicParams& cost)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    id_to_cost_[id] = cost;
  }

  CostHeuristicParams getCostById(uint8_t id) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = id_to_cost_.find(id);
    return (it != id_to_cost_.end()) ? it->second
                                     : CostHeuristicParams{0, 0, 0, 0, false};
  }

  // FIX: mutex_ now held — comment previously claimed no lock was needed,
  // which was incorrect given concurrent writes via updateCostById().
  bool hasClassId(uint8_t id) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return id_to_cost_.find(id) != id_to_cost_.end();
  }

  void updateCostByName(const std::string& name, const CostHeuristicParams& cost)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = name_to_id_.find(name);
    if (it != name_to_id_.end())
      id_to_cost_[it->second] = cost;
  }

  CostHeuristicParams getCostByName(const std::string& name) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto nit = name_to_id_.find(name);
    if (nit == name_to_id_.end())
      return CostHeuristicParams{0, 0, 0, 0, false};
    auto cit = id_to_cost_.find(nit->second);
    return (cit != id_to_cost_.end()) ? cit->second
                                      : CostHeuristicParams{0, 0, 0, 0, false};
  }

  bool empty()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return name_to_id_.empty() || id_to_cost_.empty();
  }

private:
  mutable std::mutex                               mutex_;
  std::unordered_map<std::string, uint8_t>         name_to_id_;
  std::unordered_map<uint8_t, CostHeuristicParams> id_to_cost_;
};

namespace nav2_costmap_2d {

/**
 * @class SegmentationBuffer
 * @brief Transforms point clouds to the global frame and buffers semantic
 *        segmentation observations in a SegmentationTileMap.
 *
 * Thread-safety contract:
 *   All methods that read or write segmentation_cost_multimap_ or
 *   temporal_tile_map_ must be called while holding lock_ (getMutex()).
 *   The only exception is isCurrent() which only reads clock state.
 */
class SegmentationBuffer
{
public:
  using SharedPtr = std::shared_ptr<SegmentationBuffer>;

  SegmentationBuffer(
    const nav2_util::LifecycleNode::WeakPtr&                      parent,
    std::string                                                    buffer_source,
    std::vector<std::string>                                       class_types,
    std::unordered_map<std::string, CostHeuristicParams>           class_names_cost_map,
    std::unordered_map<std::string, std::vector<std::string>>      class_type_to_names,
    double observation_keep_time,
    double expected_update_rate,
    double max_lookahead_distance,
    double min_lookahead_distance,
    tf2_ros::Buffer&   tf2_buffer,
    std::string        global_frame,
    std::string        sensor_frame,
    tf2::Duration      tf_tolerance,
    double             costmap_resolution,
    double             tile_map_decay_time,
    bool               visualize_tile_map = false,
    bool               use_cost_selection = true);

  ~SegmentationBuffer();

  /**
   * @brief Transform cloud to global frame and store observations.
   *        MUST be called while holding getMutex().
   */
  void bufferSegmentation(const sensor_msgs::msg::PointCloud2& cloud,
                          const sensor_msgs::msg::Image&        segmentation,
                          const sensor_msgs::msg::Image&        confidence);

  std::unordered_map<std::string, CostHeuristicParams> getClassMap();

  /**
   * @brief Rebuild the cost multimap from a LabelInfo message.
   *        MUST be called while holding getMutex().
   */
  void createSegmentationCostMultimap(const vision_msgs::msg::LabelInfo& label_info);

  /**
   * @brief Returns true if no LabelInfo has been received yet.
   *        MUST be called while holding getMutex().
   */
  bool isClassIdCostMapEmpty() { return segmentation_cost_multimap_->empty(); }

  bool isCurrent() const;

  inline void lock()   { lock_.lock(); }
  inline void unlock() { lock_.unlock(); }
  inline std::recursive_mutex& getMutex() { return lock_; }

  void resetLastUpdated();

  std::string              getBufferSource()  { return buffer_source_; }
  std::vector<std::string> getClassTypes()    { return class_types_; }
  std::vector<std::string> getClassNamesForType(const std::string& class_type);

  void setMinObstacleDistance(double distance)
  { sq_min_lookahead_distance_ = std::pow(distance, 2); }

  void setMaxObstacleDistance(double distance)
  { sq_max_lookahead_distance_ = std::pow(distance, 2); }

  /**
   * @brief Update cost params for a named class.
   *        MUST be called while holding getMutex().
   */
  void updateClassMap(std::string new_class, CostHeuristicParams new_cost);

  SegmentationTileMap::SharedPtr getSegmentationTileMap()
  { return temporal_tile_map_; }

  /**
   * @brief Retrieve cost params by class ID.
   *        MUST be called while holding getMutex().
   */
  CostHeuristicParams getCostForClassId(uint8_t class_id)
  { return segmentation_cost_multimap_->getCostById(class_id); }

  /**
   * @brief Retrieve cost params by class name.
   *        MUST be called while holding getMutex().
   */
  CostHeuristicParams getCostForClassName(std::string class_name)
  { return segmentation_cost_multimap_->getCostByName(class_name); }

private:
  void purgeStaleSegmentations();

  rclcpp::Clock::SharedPtr  clock_;
  rclcpp::Logger            logger_{rclcpp::get_logger("nav2_costmap_2d")};
  tf2_ros::Buffer&          tf2_buffer_;
  std::vector<std::string>  class_types_;
  std::unordered_map<std::string, CostHeuristicParams>          class_names_cost_map_;
  std::unordered_map<std::string, std::vector<std::string>>     class_type_to_names_;
  const rclcpp::Duration    observation_keep_time_;
  const rclcpp::Duration    expected_update_rate_;
  rclcpp::Time              last_updated_;
  std::string               global_frame_;
  std::string               sensor_frame_;
  std::string               buffer_source_;
  std::recursive_mutex      lock_;
  double                    sq_max_lookahead_distance_;
  double                    sq_min_lookahead_distance_;
  tf2::Duration             tf_tolerance_;

  SegmentationCostMultimap::SharedPtr segmentation_cost_multimap_;
  SegmentationTileMap::SharedPtr      temporal_tile_map_;

  bool visualize_tile_map_ = false;
  bool use_cost_selection_ = true;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tile_map_pub_;
};

}  // namespace nav2_costmap_2d
#endif  // SEMANTIC_SEGMENTATION_LAYER__SEGMENTATION_BUFFER_HPP_
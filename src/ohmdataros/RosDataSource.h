// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMDATAROS_ROSDATASOURCE_H
#define OHMDATAROS_ROSDATASOURCE_H

#include "ohmdataros/DataRosConfig.h"

#include <ohmapp/DataSource.h>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <ros/ros.h>

#include <glm/gtc/quaternion.hpp>

#include <functional>
#include <memory>
#include <mutex>
#include <vector>

namespace tf2_ros
{
class Buffer;
class TransformListener;
}  // namespace tf2_ros

namespace ohmdataros
{
class PointCloud2Helper;

class RosDataSource : public ohmapp::DataSource
{
public:
  /// Base class alias.
  using Super = ohmapp::DataSource;

  /// Command line options.
  struct Options : public Super::Options
  {
    /// List of point cloud2 topics to subscribe to. Nominally one per sensor.
    std::vector<std::string> sensor_frames_and_topics;
    /// The map frame.
    std::string map_frame = "odom";
    /// Extents to limit the in memory map to (approximate). Data outside these sensor centric extents may be discarded.
    ///
    /// The extents define an axis aligned bounding box (AABB) around the first sensor position. The size each AABB side
    /// equals twice the corresponding axis in this field.
    double map_half_extents = 10;
    /// Vertical half extents for constraining the map. Only used when @c allow_vertical_extents is set.
    double map_half_extents_vertical = 10;
    /// Minimum number of points to accumulate before triggering a batch update. Can be used to affect GPU occupancy.
    /// Only supported when @p allow_batch_size is true, which in turn is only allowed when @c DataSource::samplesOnly()
    /// is false. This is because the sensor position cannot be correctly resolved when batching multiple sensors.
    unsigned min_batch_size = 0;
    /// Frequency with which to publish the map on the topic (Hz).
    float publish_frequency = 1.0;
    /// Prevent ray truncation before submitting to a batch? Normally we truncate rays to the @c map_half_extents before
    /// submitting a batch. This option prevents truncation submitting the full rays.
    bool no_ray_truncation = false;
    /// True to automatically terminate once there are no more publishers for the @c Sensor::subscriber objects.
    bool auto_finish = false;

    /// Should we add the 'batch-size' command line option? This is not a command line option. Set to match
    /// @c !DataSource::samplesOnly() .
    bool allow_batch_size = true;
    /// Are vertical map constraints supported?
    bool allow_vertical_extents = false;

    void configure(cxxopts::OptionAdder &adder) override;
    void print(std::ostream &out) override;
  };

  /// Defines an input sensor and tracks working data. Internal use, but public for use with free functions.
  struct Sensor
  {
    /// Last ROS transform for the sensor in the map frame.
    geometry_msgs::TransformStamped ros_transform;
    /// Topic point cloud sensor messages come in on.
    std::string topic;
    /// Frame in which the sensor resides.
    std::string frame;
    /// Last extracted position for the sensor in the map frame.
    glm::dvec3 position;
    /// Last extracted rotation for the sensor in the map frame.
    glm::dquat rotation;
    /// Helper object for extracting point cloud data.
    std::shared_ptr<PointCloud2Helper> scan_reader;
    /// Messages pending processing. These accumulate as we wait for transform data.
    std::vector<sensor_msgs::PointCloud2::ConstPtr> buffered_clouds;
    /// Subscriber object to the topic
    ros::Subscriber subscriber;
    /// True once we've received any data on the topic.
    bool has_received = false;
  };

  /// Default constructor.
  RosDataSource(bool allow_vertical_extents = false);
  /// Destructor.
  ~RosDataSource();

  void setSamplesOnly(bool samples_only) override;

  inline Options &options() { return static_cast<Options &>(Super::options()); }
  inline const Options &options() const { return static_cast<const Options &>(Super::options()); }

  std::string sourceName() const override;

  inline void setPublishCloudFunction(std::function<void()> callback) { publish_cloud_function_ = callback; }
  inline const std::function<void()> &publishCloudFunction() const { return publish_cloud_function_; }

  glm::dvec3 sensorPosition(unsigned sensor_index = 0) const;
  glm::dquat sensorRotation(unsigned sensor_index = 0) const;

  uint64_t processedPointCount() const override;
  double processedTimeRange() const override;
  unsigned expectedBatchSize() const override;
  void requestBatchSettings(unsigned batch_size, double max_sensor_motion) override;

  inline ros::NodeHandle &nodeHandle() const { return *node_handle_; }

  int validateOptions() override;
  int prepareForRun(uint64_t &point_count, const std::string &reference_name) override;

  int run(BatchFunction batch_function, unsigned *quit_level_ptr) override;

protected:
  /// Return codes for @c processCloud()
  enum class ProcessResult
  {
    kFailed,      ///< Failed - can't process cloud.
    kOk,          ///< Processing completed.
    kNoTransform  ///< Unable to resolve sensor transform (yet).
  };

  /// Buffer for incoming data. Samples are marshalled in the buffer from a @c PointCloud2 message for processing.
  ///
  /// If @c DataSource::samplesOnly() is true, then the @c sensors_and_samples buffer only contains sample points and
  /// batches are processed immediately on receipt. Otherwise @c sensors_and_samples is interlaced with sensor/sample
  /// position pairs and we can support delayed batch processing.
  struct RaysBuffer
  {
    /// Sensor/sample position pairs when @c DataSource::samplesOnly() is @c false or just sample poisitions when
    /// @c true .
    std::vector<glm::dvec3> sensors_and_samples;
    /// Timestamps for each sample in @c sensors_and_samples .
    std::vector<double> timestamps;
    /// Intensity values for each sample in @c sensors_and_samples . Zero when unknown.
    std::vector<float> intensities;
    /// Colour values for each sample in @c sensors_and_samples . Black when unknown.
    std::vector<glm::vec4> colours;

    /// Clear the buffers.
    inline void clear()
    {
      sensors_and_samples.clear();
      timestamps.clear();
      intensities.clear();
      colours.clear();
    }

    /// Query the number of samples in the buffer.
    inline size_t size() const { return timestamps.size(); }
    /// Is the buffer empty.
    inline bool empty() const { return timestamps.empty(); }
  };

  void updateSensorTransform(unsigned sensor_index);
  void onCloudReceive(const sensor_msgs::PointCloud2::ConstPtr &cloud, unsigned sensor_index);
  ProcessResult processCloud(const sensor_msgs::PointCloud2::ConstPtr &cloud, unsigned sensor_index);
  void processBufferedClouds(unsigned sensor_index);

  void addToRayBuffer(RaysBuffer &rays, const glm::dvec3 &sensor, const glm::dvec3 &sample, double timestamp,
                      float intensity, const glm::vec4 &colour);

  void processRaysAndClear(RaysBuffer &rays, const glm::dvec3 &sensor_position);

  void onPublishTimer(const ros::TimerEvent &event);

  /// Check to see if publishing on the sensor topics appears to have ended.
  ///
  /// Requires that for each topic:
  /// - we have received some data
  /// - there are no longer any publishers.
  /// Returning true if all conditions on all sensors are met.
  ///
  /// @return True if it looks like there's no more data incoming.
  bool publishingFinished() const;

  /// Retrieve the global stats.
  Stats globalStats() const override;

  /// Retrieve the windowed stats.
  Stats windowedStats() const override;

private:
  /// Update @c windowedStats().
  void updateWindowedStats();

  /// Collate @p stats with the windowed and global stats.
  /// @param stats The stats for the last data batch.
  void addBatchStats(const Stats &stats);

  std::unique_ptr<ros::NodeHandle> node_handle_;
  std::vector<Sensor> sensors_;
  std::atomic<uint64_t> processed_point_count_{};
  std::atomic<double> processed_time_range_{};
  BatchFunction batch_function_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  /// First processed message timestamp.
  double first_timestamp_ = -1;
  /// Last processed message timestamp.
  double last_timestamp_ = -1;
  RaysBuffer rays_buffer_;
  /// Stats for data in @c rays_buffer_.
  Stats rays_buffer_stats_;
  std::function<void()> publish_cloud_function_;
  /// Stats window ring buffer.
  std::vector<Stats> windowed_stats_buffer_;
  /// Target buffer size for the stats window ring buffer.
  unsigned windowed_stats_buffer_size_ = 20;
  /// Next insertion index into the @c windowed_stats_buffer_ ring buffer.
  unsigned windowed_stats_buffer_next_ = 0;
  /// Global data stats.
  Stats global_stats_;
  /// Windowed data stats.
  Stats windowed_stats_;
  /// Access guard for @c windowed_stats_
  mutable std::mutex stats_lock_;
  ros::Timer publish_timer_;
  bool finish_ = false;
  /// CSV logging stream for stats.
  std::unique_ptr<std::ostream> stats_csv_;
};
}  // namespace ohmdataros

#endif  // OHMDATAROS_ROSDATASOURCE_H

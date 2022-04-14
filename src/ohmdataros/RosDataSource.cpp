// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "RosDataSource.h"

#include "PointCloud2Helper.h"

#include <ohm/Logger.h>

#include <slamio/SlamCloudLoader.h>

#include <glm/ext.hpp>
#include <glm/glm.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/normal.hpp>

#include <ros/ros.h>
#include <ros/time.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <fstream>
#include <sstream>

// Must be after argument streaming operators.
#include <ohmutil/Options.h>

using Clock = std::chrono::high_resolution_clock;

namespace
{
/// Return values for @c lookupTransform()
enum LookupTransformResult : int
{
  kLtrOk = 0,
  kLtrLookupFailure,
  kLtrTransformFailure
};


/// Lookup a ROS frame transformation, optionally at a given time. This unction wraps catching the exceptions
/// to yield a return value instead.
LookupTransformResult lookupTransform(geometry_msgs::TransformStamped *transform, const tf2_ros::Buffer &tf_buffer,
                                      const std::string &target_frame, const std::string &src_frame,
                                      const ros::Time &time = ros::Time(0))
{
  if (target_frame == src_frame)
  {
    transform->transform.translation.x = transform->transform.translation.y = transform->transform.translation.z = 0;
    transform->transform.rotation.w = 1;
    transform->transform.rotation.x = transform->transform.rotation.y = transform->transform.rotation.z = 0;
    return kLtrOk;
  }

  // Lookup the transform buffer for an appropriate transform.
  try
  {
    *transform = tf_buffer.lookupTransform(target_frame, src_frame, time);
    return kLtrOk;
  }
  catch (const tf2::LookupException &e)
  {
    // Transform not present yet. Stop trying for this data set.
    ROS_WARN_THROTTLE(1.0, e.what());
    return kLtrLookupFailure;
  }
  catch (const tf2::ExtrapolationException &e)
  {
    // Too soon or too late. Too late we want to drop, but too soon we can keep. Unfortunately, the same exception is
    // used, so we differentiate by the message stating that extrapolation is required into the future (vs past).
    if (std::string(e.what()).find("future") != std::string::npos)
    {
      // Too soon.
      return kLtrTransformFailure;
    }
    ROS_WARN_THROTTLE(1.0, e.what());
  }
  catch (const tf2::TransformException &e)
  {
    // Different issues. Skip the point.
    ROS_WARN_THROTTLE(1.0, e.what());
  }

  return kLtrLookupFailure;
}


/// Helper function to map from a @c geometry_msgs::TransformStamped to a @c glm::mat4
glm::mat4 geomMsgToMat4(const geometry_msgs::TransformStamped &src)
{
  glm::mat4 mat = glm::translate(glm::dmat4(1.0), glm::dvec3(src.transform.translation.x, src.transform.translation.y,
                                                             src.transform.translation.z)) *
                  glm::mat4_cast(glm::dquat(src.transform.rotation.w, src.transform.rotation.x,
                                            src.transform.rotation.y, src.transform.rotation.z));
  return mat;
}
}  // namespace

namespace ohmdataros
{
void RosDataSource::Options::configure(cxxopts::OptionAdder &adder)
{
  Super::Options::configure(adder);
  // clang-format off
  adder
    ("auto-finish", "Try to detect completion automatically and exit.", optVal(auto_finish))
    ("map-frame", "Frame in which the map is generated.", optVal(map_frame))
    ("map-half-extents", "Half AABB extents of the local map region to maintain.", optVal(map_half_extents))
    // ("no-ray-truncation", "Prevent ray truncation before submitting to a batch? Normally we truncate rays to the "
    //                       "map-half-extents before submitting a batch. This option prevents truncation submitting the "
    //                       "full rays.", optVal(no_ray_truncation))
    ("publish-frequency", "Frequency with which to publish map data messages (Hz).", optVal(publish_frequency))
    ("sensor-topics", "Sensor [frame:]topic. Lists the PointCloud2 topics, optionally prefixed by name of the frame in "
                      "which the sensor generating this topic resides - e.g., \"sensor_frame:/velodyne_points\". "
                      "Assumes sensor is in the map frame when not specified.", cxxopts::value(sensor_frames_and_topics))
    ;

  if (allow_batch_size)
  {
    adder
      ("batch-size", "Minimum number of points to accumulate before triggering a batch update. Batches are always triggerd with all points from pending incoming messages.", optVal(min_batch_size))
      ;
  }

  if (allow_vertical_extents)
  {
    adder
      ("map-half-extents-z", "Vertical constraints for the half AABB of the local map.", optVal(map_half_extents_vertical))
      ;
  }
  // clang-format on
}


void RosDataSource::Options::print(std::ostream &out)
{
  std::ostringstream topics;
  std::string separator = " ";
  for (auto &&topic : sensor_frames_and_topics)
  {
    topics << separator << topic;
    separator = ",";
  }
  out << "Cloud topics: " << topics.str() << '\n';
  out << "Map frame: " << map_frame << '\n';
  out << "Max half extents: " << map_half_extents << '\n';
  out << "Public frequency: " << publish_frequency << '\n';
  out << "Auto finish: " << auto_finish << '\n';
  Super::Options::print(out);
}

RosDataSource::RosDataSource(bool allow_vertical_extents)
  : Super(std::make_unique<Options>())
  , node_handle_(std::make_unique<ros::NodeHandle>("~"))
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  options().allow_vertical_extents = allow_vertical_extents;
}


RosDataSource::~RosDataSource()
{
  tf_listener_.release();
  tf_buffer_.release();
  sensors_.clear();
  node_handle_.release();
}


void RosDataSource::setSamplesOnly(bool samples_only)
{
  Super::setSamplesOnly(samples_only);
  options().allow_batch_size = !samplesOnly();
}


std::string RosDataSource::sourceName() const
{
  return (!options().sensor_frames_and_topics.empty()) ? options().sensor_frames_and_topics.front() : "rosdata";
}


glm::dvec3 RosDataSource::sensorPosition(unsigned sensor_index) const
{
  if (sensor_index < sensors_.size())
  {
    return sensors_[sensor_index].position;
  }
  return glm::dvec3(0.0);
}


glm::dquat RosDataSource::sensorRotation(unsigned sensor_index) const
{
  if (sensor_index < sensors_.size())
  {
    return sensors_[sensor_index].rotation;
  }
  return glm::dquat(1.0, 0.0, 0.0, 0.0);
}


uint64_t RosDataSource::processedPointCount() const
{
  return processed_point_count_;
}


double RosDataSource::processedTimeRange() const
{
  return processed_time_range_;
}


unsigned RosDataSource::expectedBatchSize() const
{
  return 0;
}


void RosDataSource::requestBatchSettings(unsigned batch_size, double max_sensor_motion)
{
  options().min_batch_size = batch_size;
  // Not supported. Based on incoming topic.
  (void)max_sensor_motion;
}


int RosDataSource::validateOptions()
{
  if (options().sensor_frames_and_topics.empty())
  {
    ohm::logger::error("Missing point cloud topic\n");
    return -1;
  }

  return 0;
}


int RosDataSource::prepareForRun(uint64_t &point_count, const std::string &reference_name)
{
  point_count = 0;  // Not known.

  const unsigned topic_count = unsigned(options().sensor_frames_and_topics.size());
  sensors_.clear();
  sensors_.reserve(topic_count);
  for (unsigned i = 0; i < topic_count; ++i)
  {
    const auto &sensor_frame_and_topic = options().sensor_frames_and_topics[i];

    sensors_.emplace_back(Sensor{});
    Sensor &sensor = sensors_.back();
    std::string::size_type split_pos = 0;
    if ((split_pos = sensor_frame_and_topic.find(":")) == std::string::npos)
    {
      sensor.frame = options().map_frame;
      sensor.topic = sensor_frame_and_topic;
    }
    else
    {
      sensor.frame = sensor_frame_and_topic.substr(0, split_pos);
      sensor.topic = sensor_frame_and_topic.substr(split_pos + 1);
    }
    ROS_INFO("Subscribe to %s from sensor in frame %s", sensor.topic.c_str(), sensor.frame.c_str());
    const boost::function<void(const sensor_msgs::PointCloud2::ConstPtr &)> cloud_callback =
      boost::bind(&RosDataSource::onCloudReceive, this, _1, i);
    sensor.scan_reader = std::make_shared<PointCloud2Helper>();
    sensor.subscriber = node_handle_->subscribe(sensor.topic, 20, cloud_callback);
  }

  if (!reference_name.empty() && options().stats_mode == StatsMode::Csv)
  {
    const auto csv_stats_file = reference_name + "_stats.csv";
    stats_csv_ = std::make_unique<std::ofstream>(csv_stats_file.c_str());
    Stats::writeCsvHeader(*stats_csv_);
  }

  return 0;
}


int RosDataSource::run(BatchFunction batch_function, unsigned *quit_level_ptr)
{
  // Set a timer to call
  const double publish_interval = (options().publish_frequency > 0) ? 1.0 / options().publish_frequency : 1.0;
  publish_timer_ = node_handle_->createTimer(ros::Duration(publish_interval), &RosDataSource::onPublishTimer, this);

  // Configure the loop rate to run at least twice as fast as the publishing frequency.
  ros::Rate loop_rate(std::max(100.0, 2 * publish_interval));

  // Bind the processing callback.
  batch_function_ = batch_function;
  // Setup a quit flag which will be set based on the return value from batch_function.
  finish_ = false;
  // Spin and loop, until either ROS quits or finish_ is set to true based on the return value from batch_function.
  while (!finish_ && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    finish_ = finish_ || (options().auto_finish && publishingFinished()) || (quit_level_ptr && *quit_level_ptr != 0u);
  }

  if (stats_csv_)
  {
    stats_csv_->flush();
    stats_csv_.reset();
  }

  return 0;
}


void RosDataSource::updateSensorTransform(unsigned sensor_index)
{
  Sensor &sensor = sensors_[sensor_index];
  if (::lookupTransform(&sensor.ros_transform, *tf_buffer_, options().map_frame, sensor.frame) == kLtrOk)
  {
    sensor.position.x = sensor.ros_transform.transform.translation.x;
    sensor.position.y = sensor.ros_transform.transform.translation.y;
    sensor.position.z = sensor.ros_transform.transform.translation.z;
    sensor.rotation.w = sensor.ros_transform.transform.rotation.w;
    sensor.rotation.x = sensor.ros_transform.transform.rotation.x;
    sensor.rotation.y = sensor.ros_transform.transform.rotation.y;
    sensor.rotation.z = sensor.ros_transform.transform.rotation.z;
  }

  // // Update the local clipping AABB to be on the latest sensor origin (before the adjustment below)
  // const glm::dvec3 clip_half_ext(imp_->params.sensors[sensor_index].ray_half_extents.x,
  //                                imp_->params.sensors[sensor_index].ray_half_extents.y,
  //                                imp_->params.sensors[sensor_index].ray_half_extents.z);
  // imp_->local_map_aabb = ohm::Aabb(imp_->last_sensor_pos - clip_half_ext, imp_->last_sensor_pos + clip_half_ext);
}


void RosDataSource::onCloudReceive(const sensor_msgs::PointCloud2::ConstPtr &cloud, unsigned sensor_index)
{
  sensors_[sensor_index].has_received = true;
  // Get most up to date sensor position.
  updateSensorTransform(sensor_index);

  // Process buffered points which haven't been correlated with the sensor position. This will drop very old points.
  processBufferedClouds(sensor_index);

  if (processCloud(cloud, sensor_index) == ProcessResult::kNoTransform)
  {
    sensors_[sensor_index].buffered_clouds.emplace_back(cloud);
  }
}

RosDataSource::ProcessResult RosDataSource::processCloud(const sensor_msgs::PointCloud2::ConstPtr &cloud,
                                                         unsigned sensor_index)
{
  // Process new points
  Sensor &sensor = sensors_[sensor_index];
  auto &scan_reader = *sensor.scan_reader;

  // Decode and cache the cloud.
  if (!scan_reader.initialised())
  {
    if (!scan_reader.init(cloud->fields.data(), unsigned(cloud->fields.size()), cloud->is_bigendian))
    {
      ROS_ERROR("ohmdataros: Invalid initial point cloud fields.");
      return ProcessResult::kFailed;
    }
  }
  else if (!scan_reader.validate(cloud->fields.data(), unsigned(cloud->fields.size()), cloud->is_bigendian))
  {
    ROS_ERROR("ohmdataros: Point cloud field mismatch.");
    return ProcessResult::kFailed;
  }

  // Process the scan.
  geometry_msgs::TransformStamped sensor_to_map_frame, cloud_to_map_frame;
  const uint8_t *point_data = nullptr;
  glm::dvec3 sample{};
  glm::dvec3 sensor_position{};
  glm::vec4 colour{};
  double timestamp{};
  float intensity{};

  // Resolve how stale the message is. We drop messages which arrive time T before the latest transform message for this
  // sensor where the difference between T and the latest sensor message time is proportional to the tf_buffer_ length.
  // Warning: convert toSec() before doing the subtraction. Otherwise is may crash.
  const double latest_sensor_time = sensor.ros_transform.header.stamp.toSec();
  const double buffer_time = tf_buffer_->getCacheLength().toSec() * 2;
  const double drop_before_time = latest_sensor_time - buffer_time;
  const double message_time = cloud->header.stamp.toSec();

  const bool has_time_field = scan_reader.timeCount();
  const bool has_intensity_field = scan_reader.intensityCount();
  const bool has_colour_field = scan_reader.colour();

  if (message_time < drop_before_time)
  {
    ROS_ERROR("Dropping %u points which came too late", cloud->height * cloud->width);
    return ProcessResult::kFailed;
  }

  // Resolve the sensor position from the sensor frame at the cloud message time.
  auto lookup_result =
    ::lookupTransform(&sensor_to_map_frame, *tf_buffer_, options().map_frame, sensor.frame, cloud->header.stamp);
  if (lookup_result == kLtrOk)
  {
    glm::dvec3 sensor_position{};
    sensor_position.x = sensor_to_map_frame.transform.translation.x;
    sensor_position.y = sensor_to_map_frame.transform.translation.y;
    sensor_position.z = sensor_to_map_frame.transform.translation.z;

    // Get the sensor transform at the message time.
    lookup_result = ::lookupTransform(&cloud_to_map_frame, *tf_buffer_, options().map_frame, cloud->header.frame_id,
                                      cloud->header.stamp);
    if (lookup_result == kLtrOk)
    {
      glm::dmat4 cloud_to_map_transform = geomMsgToMat4(cloud_to_map_frame);

      const double message_timestamp = cloud->header.stamp.toSec();
      // We have a transform for the sensor. We can transform the points into the map frame.
      while (scan_reader.nextPoint(&point_data, cloud))
      {
        sample.x = scan_reader.x(point_data);
        sample.y = scan_reader.y(point_data);
        sample.z = scan_reader.z(point_data);
        timestamp = (has_time_field) ? scan_reader.time(point_data) : message_timestamp;
        intensity = (has_intensity_field) ? scan_reader.intensity(point_data) : 0;
        colour.x = (has_colour_field) ? scan_reader.red(point_data) : 0;
        colour.y = (has_colour_field) ? scan_reader.green(point_data) : 0;
        colour.z = (has_colour_field) ? scan_reader.blue(point_data) : 0;
        colour.w = 1.0f;
        sample = cloud_to_map_transform * glm::dvec4(glm::dvec3(sample), 1.0);
        addToRayBuffer(rays_buffer_, sensor_position, sample, timestamp, intensity, colour);
      }

      if (samplesOnly() || rays_buffer_.size() >= options().min_batch_size)
      {
        processRaysAndClear(rays_buffer_, sensor_position);
      }

      return ProcessResult::kOk;
    }
    else if (lookup_result == kLtrLookupFailure)
    {
      ROS_WARN_THROTTLE(1.0, "Cloud frame lookup failed : %s -> %s", cloud->header.frame_id.c_str(),
                        options().map_frame.c_str());
      return ProcessResult::kFailed;
    }
  }
  else if (lookup_result == kLtrLookupFailure)
  {
    ROS_WARN_THROTTLE(1.0, "Sensor frame lookup failed : %s -> %s", sensor.frame.c_str(), options().map_frame.c_str());
    return ProcessResult::kFailed;
  }

  return ProcessResult::kNoTransform;
}


void RosDataSource::processBufferedClouds(unsigned sensor_index)
{
  Sensor &sensor = sensors_[sensor_index];
  size_t erase_up_to_index = 0;
  for (size_t i = 0; i < sensor.buffered_clouds.size(); ++i)
  {
    auto result = processCloud(sensor.buffered_clouds[i], sensor_index);
    if (result == ProcessResult::kNoTransform)
    {
      // Too soon to resolve a transform for this message. Stop processessing. We assume messages are monotonic in time
      // and we won't be able to resolve any *later* message transforms.
      break;
    }

    // Drop buffers so long as the result is failed (cannot process) or ok (processed).
    ++erase_up_to_index;
  }

  if (erase_up_to_index)
  {
    // Move clouds from higher in the array.
    const size_t preserve_count = sensor.buffered_clouds.size() - erase_up_to_index;
    for (size_t i = 0; i < preserve_count; ++i)
    {
      sensor.buffered_clouds[i] = sensor.buffered_clouds[erase_up_to_index + i];
    }
    // Drop items after erase_up_to_index
    sensor.buffered_clouds.resize(preserve_count);
  }
}


void RosDataSource::addToRayBuffer(RaysBuffer &rays, const glm::dvec3 &sensor, const glm::dvec3 &sample,
                                   double timestamp, float intensity, const glm::vec4 &colour)
{
  // Check validity
  const glm::dvec3 ray = sample - sensor;
  const double ray_length_squared = glm::dot(ray, ray);  // Length squared.

  if (first_timestamp_ < 0)
  {
    first_timestamp_ = global_stats_.data_time_start = rays_buffer_stats_.data_time_start = timestamp;
    global_stats_.process_time_start = rays_buffer_stats_.process_time_start = ros::Time::now().toSec();
  }

  // Sanitise.
  if (!std::isnan(ray_length_squared) && !std::isinf(ray_length_squared))
  {
    if (!samplesOnly())
    {
      rays.sensors_and_samples.emplace_back(sensor);
    }
    rays.sensors_and_samples.emplace_back(sample);
    rays.timestamps.emplace_back(timestamp);
    rays.intensities.emplace_back(intensity);
    rays.colours.emplace_back(colour);

    const double ray_length = std::sqrt(ray_length_squared);
    // Don't update rays_buffer_stats_.time_start. That is managed in processRaysAndClear()
    rays_buffer_stats_.data_time_end = std::max(timestamp, rays_buffer_stats_.data_time_end);
    rays_buffer_stats_.ray_length_minimum = std::min(ray_length, rays_buffer_stats_.ray_length_minimum);
    rays_buffer_stats_.ray_length_maximum = std::max(ray_length, rays_buffer_stats_.ray_length_maximum);
    rays_buffer_stats_.ray_length_total += ray_length;
    ++rays_buffer_stats_.ray_count;
  }
}


void RosDataSource::onPublishTimer(const ros::TimerEvent &event)
{
  (void)event;
  // Try process buffered messages and trigger old batches. This ensures we don't have points sitting in the buffer if
  // there is a break in the sensor messages.
  for (unsigned i = 0; i < unsigned(sensors_.size()); ++i)
  {
    processBufferedClouds(i);
  }
  if (!samplesOnly())
  {
    if (!rays_buffer_.empty() && rays_buffer_.size() > options().min_batch_size)
    {
      processRaysAndClear(rays_buffer_, rays_buffer_.sensors_and_samples[0]);
    }
  }

  if (publish_cloud_function_)
  {
    publish_cloud_function_();
  }
}


void RosDataSource::processRaysAndClear(RaysBuffer &rays, const glm::dvec3 &sensor_position)
{
  finish_ =
    !batch_function_(sensor_position, rays.sensors_and_samples, rays.timestamps, rays.intensities, rays.colours);
  if (!rays.empty())
  {
    rays_buffer_stats_.process_time_end = ros::Time::now().toSec();
    if (options().stats_mode != StatsMode::Off)
    {
      addBatchStats(rays_buffer_stats_);
    }
    processed_point_count_ += rays.size();
    processed_time_range_ = last_timestamp_ - first_timestamp_;
    last_timestamp_ = rays.timestamps.back();
    rays_buffer_stats_.reset(rays_buffer_stats_.process_time_end, rays_buffer_stats_.data_time_end);
  }
  rays.clear();
}


bool RosDataSource::publishingFinished() const
{
  for (const auto &sensor : sensors_)
  {
    if (!sensor.has_received)
    {
      return false;
    }

    if (sensor.subscriber.getNumPublishers() > 0)
    {
      return false;
    }
  }

  return true;
}


ohmapp::DataSource::Stats RosDataSource::globalStats() const
{
  return global_stats_;
}


ohmapp::DataSource::Stats RosDataSource::windowedStats() const
{
  std::unique_lock<std::mutex> guard(stats_lock_);
  return windowed_stats_;
}


void RosDataSource::updateWindowedStats()
{
  Stats stats{};
  calculateWindowedStats(stats, windowed_stats_buffer_.begin(), windowed_stats_buffer_.end());
  std::unique_lock<std::mutex> guard(stats_lock_);
  windowed_stats_ = stats;
  guard.unlock();
  if (stats_csv_)
  {
    // Normalise the process time for better display.
    stats.process_time_start -= global_stats_.process_time_start;
    stats.process_time_end -= global_stats_.process_time_start;
    *stats_csv_ << stats << '\n';
  }
}


void RosDataSource::addBatchStats(const Stats &stats)
{
  windowed_stats_buffer_next_ = DataSource::addBatchStats(stats, global_stats_, windowed_stats_buffer_,
                                                          windowed_stats_buffer_size_, windowed_stats_buffer_next_);

  // Update the windowed stats whenever the buffer is full.
  if (windowed_stats_buffer_next_ == 0)
  {
    updateWindowedStats();
  }
}
}  // namespace ohmdataros

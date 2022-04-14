// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef OHMPOPROS_ROSARGMAP_H
#define OHMPOPROS_ROSARGMAP_H

#include <ros/ros.h>

#include <map>
#include <string>
#include <vector>

namespace ohmpopros
{
/// Set of ros param values to mirgate to command line arguments.
inline std::map<std::string, std::string> paramToArgMappingsCommon()
{
  std::map<std::string, std::string> param_to_arg =  //
    {
      // ROS data source args
      { "auto_finish", "auto-finish" },
      { "batch_size", "batch-size" },
      { "map_frame", "map-frame" },
      { "map_half_extents", "map-half-extents" },
      { "map_half_extents_z", "map-half-extents-z" },
      { "publish_frequency", "publish-frequency" },
      { "sensor_topics", "sensor-topics" },
      { "stats", "stats" },
      // Final output options
      { "cloud_colour", "cloud-colour" },
      { "output", "output" },
      { "quiet", "quiet" },
      { "save_info", "save-info" },
      { "save_map", "save-map" },
      // map args
      { "resolution", "resolution" },
    };
  return param_to_arg;
}

/// Set of ros param values to mirgate to command line arguments.
inline std::map<std::string, std::string> paramToArgMappingsOhmCommon()
{
  std::map<std::string, std::string> param_to_arg =  //
    {
      // map args
      { "ray_length_max", "ray-length-max " },
      { "clamp", "clamp" },
      { "dim", "dim" },
      { "hit", "hit" },
      { "miss", "miss" },
      { "voxel_mean", "voxel-mean" },
      { "traversal", "traversal" },
      { "threshold", "threshold " },
      { "mode", "mode" },
      // Final output options
      { "trace", "trace" },
      { "trace_final", "trace-final" },
      // ndt args
      { "ndt", "ndt" },
      { "ndt_cov_point_threshold", "ndt-cov-point-threshold" },
      { "ndt_cov_prob_threshold", "ndt-cov-prob-threshold" },
      { "ndt_adaptation_rate", "ndt-adaptation-rate" },
      { "ndt_sensor_noise", "ndt-sensor-noise" },
      // compression args
      { "high_tide", "high-tide" },
      { "low_time", "low-time" },
      { "uncompressed", "uncompressed" },
    };
  for (auto &&pair : paramToArgMappingsCommon())
  {
    param_to_arg.insert(pair);
  }

  return param_to_arg;
}

/// Set of ros param values to mirgate to command line arguments.
inline std::map<std::string, std::string> paramToArgMappingsOhmGpuCommon()
{
  std::map<std::string, std::string> param_to_arg =  //
    {
      // Gpu common
      { "gpu_cache_size", "gpu-cache-size" },
      { "gpu_ray_segment_length", "gpu-ray-segment-length" },
    };
  return param_to_arg;
}

/// Set of ros param values to mirgate to command line arguments.
inline std::map<std::string, std::string> paramToArgMappingsOhmCpu()
{
  auto param_to_arg = paramToArgMappingsOhmCommon();
  return param_to_arg;
}

/// Set of ros param values to mirgate to command line arguments.
inline std::map<std::string, std::string> paramToArgMappingsOhmCuda()
{
  auto param_to_arg = paramToArgMappingsOhmCommon();
  for (auto &&pair : paramToArgMappingsOhmGpuCommon())
  {
    param_to_arg.insert(pair);
  }
  return param_to_arg;
}

/// Set of ros param values to mirgate to command line arguments.
inline std::map<std::string, std::string> paramToArgMappingsOhmOcl()
{
  std::map<std::string, std::string> param_to_arg =  //
    {
      //
      // OpenCL specific options
      { "accel", "accel" },         { "clver", "clver" },       { "device", "device" },
      { "gpu_debug", "gpu-debug" }, { "platform", "platform" }, { "vendor", "vendor" },
    };

  for (auto &&pair : paramToArgMappingsOhmCommon())
  {
    param_to_arg.insert(pair);
  }
  for (auto &&pair : paramToArgMappingsOhmGpuCommon())
  {
    param_to_arg.insert(pair);
  }
  return param_to_arg;
}


inline void migrateParamsToArgs(std::vector<std::string> &args, ros::NodeHandle &node,
                                const std::map<std::string, std::string> &params_to_args)
{
  for (const auto &param_to_arg : params_to_args)
  {
    std::string arg_value;
    if (node.hasParam(param_to_arg.first) && node.getParam(param_to_arg.first, arg_value))
    {
      args.emplace_back("--" + param_to_arg.second + "=" + arg_value);
    }
  }
}
}  // namespace ohmpopros

#endif  // OHMPOPROS_ROSARGMAP_H

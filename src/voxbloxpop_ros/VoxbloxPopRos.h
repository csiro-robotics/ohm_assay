// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef VOXBLOXLPOPROS_H
#define VOXBLOXLPOPROS_H

#include <voxbloxpop/VoxbloxPopMap.h>

#include <ohmdataros/MapPublish.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <ros/ros.h>

#include <map>
#include <string>

namespace voxpopros
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

inline std::map<std::string, std::string> paramToArgMappingsTsdf()
{
  std::map<std::string, std::string> param_to_arg =  //
    { { "allow_clear", "allow-clear" },
      { "anti_grazing", "anti-grazing" },
      { "dim", "dim" },
      { "max_weight", "max-weight" },
      { "mode", "mode" },
      { "ray_length_min", "ray-length-min" },
      { "ray_length_max", "ray-length-max" },
      { "sparsity_compensation", "sparsity-compensation" },
      { "surface_threshold", "surface-threshold" },
      { "threads", "threads" },
      { "trunc_dist", "trunc-dist" },
      { "use_const_weight", "use-const-weight" },
      { "use_weight_dropoff", "use-weight-dropoff" },
      { "use_sparsity_compensation", "use-sparsity-compensation" },
      { "voxel_carving", "voxel-carving" } };

  for (auto &&pair : paramToArgMappingsCommon())
  {
    param_to_arg.insert(pair);
  }

  return param_to_arg;
}

inline std::map<std::string, std::string> paramToArgMappingsOccupancy()
{
  std::map<std::string, std::string> param_to_arg =  //
    {
      { "dim", "dim" },
      { "hit", "hit" },
      { "miss", "miss" },
      { "ray_length_min", "ray-length-min" },
      { "ray_length_max", "ray-length-max" },
      { "threshold", "threshold" },
    };

  for (auto &&pair : paramToArgMappingsCommon())
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


// From voxblox_ros: ptcloud_vis.h
template <typename VoxelType>
void publishMapCloud(ros::Publisher &publisher, const std::string &map_frame, const voxblox::Layer<VoxelType> &layer,
                     voxbloxpop::ShouldVisualizeVoxelColorFunctionType<VoxelType> vis_function,
                     ohmdataros::ColourSolver colour_solver)
{
  if (publisher.getNumSubscribers() == 0)
  {
    return;
  }

  const float resolution = layer.voxel_size();
  sensor_msgs::PointCloud2 map_cloud;
  map_cloud.header.frame_id = map_frame;
  map_cloud.header.stamp = ros::Time::now();

  // Temp variables.
  voxblox::Color color;
  voxblox::BlockIndexList blocks;
  layer.getAllAllocatedBlocks(&blocks);

  // Cache layer settings.
  size_t vps = layer.voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  // iteration variables
  auto block_index_iter = blocks.begin();
  size_t linear_index = 0;

  const auto next_point = [&](ohmdataros::Voxel &voxel) {
    while (block_index_iter != blocks.end())
    {
      const voxblox::Block<VoxelType> &block = layer.getBlockByIndex(*block_index_iter);
      while (linear_index < num_voxels_per_block)
      {
        voxblox::Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
        const auto &voxblox_voxel = block.getVoxelByLinearIndex(linear_index);
        ++linear_index;
        if (vis_function(voxblox_voxel, coord, &color))
        {
          voxel.x = coord.x();
          voxel.y = coord.y();
          voxel.z = coord.z();
          voxel.scale = resolution;
          voxel.intensity = uint16_t(voxel.r * float(0xffffu));
          const auto colour = colour_solver(voxel);
          voxel.r = colour.rf();
          voxel.g = colour.gf();
          voxel.b = colour.bf();
          voxel.a = colour.af();
          return true;
        }
      }

      // Next block. Need to reset the linear_index for this.
      linear_index = 0;
      ++block_index_iter;
    }

    return false;
  };

  ohmdataros::populateCloudMessage(map_cloud, next_point);

  publisher.publish(map_cloud);
}


template <typename VoxelType>
void publishMapVoxels(ros::Publisher &publisher, const std::string &map_frame, const voxblox::Layer<VoxelType> &layer,
                      voxbloxpop::ShouldVisualizeVoxelColorFunctionType<VoxelType> vis_function,
                      const std::string &name, ohmdataros::ColourSolver colour_solver)
{
  if (publisher.getNumSubscribers() == 0)
  {
    return;
  }

  const float resolution = layer.voxel_size();
  visualization_msgs::Marker markers;
  markers.header.frame_id = map_frame;
  markers.header.stamp = ros::Time::now();
  markers.ns = name;
  markers.id = 0;

  // Temp variables.
  voxblox::Color color;
  voxblox::BlockIndexList blocks;
  layer.getAllAllocatedBlocks(&blocks);

  // Cache layer settings.
  size_t vps = layer.voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  // iteration variables
  auto block_index_iter = blocks.begin();
  size_t linear_index = 0;

  const auto next_point = [&](ohmdataros::Voxel &voxel) {
    while (block_index_iter != blocks.end())
    {
      const voxblox::Block<VoxelType> &block = layer.getBlockByIndex(*block_index_iter);
      while (linear_index < num_voxels_per_block)
      {
        voxblox::Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
        const auto &voxblox_voxel = block.getVoxelByLinearIndex(linear_index);
        ++linear_index;
        if (vis_function(voxblox_voxel, coord, &color))
        {
          voxel.x = coord.x();
          voxel.y = coord.y();
          voxel.z = coord.z();
          voxel.scale = resolution;
          voxel.intensity = uint16_t(voxel.r * float(0xffffu));
          const auto colour = colour_solver(voxel);
          voxel.r = colour.rf();
          voxel.g = colour.gf();
          voxel.b = colour.bf();
          voxel.a = colour.af();
          return true;
        }
      }

      // Next block. Need to reset the linear_index for this.
      linear_index = 0;
      ++block_index_iter;
    }

    return false;
  };

  // Don't publish empty marker messages. Rviz doesn't like it.
  if (ohmdataros::populateMarkerMessage(markers, next_point) > 0)
  {
    publisher.publish(markers);
  }
}
}  // namespace voxpopros

#endif  // VOXBLOXLPOPROS_H

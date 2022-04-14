// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OCTOMAPPOPROS_OCTOPOPROS_H
#define OCTOMAPPOPROS_OCTOPOPROS_H

#include <octomappop/OctomapPop.h>

#include <ohmdataros/MapPublish.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <ros/ros.h>

#include <octomap/octomap.h>

#include <map>
#include <string>

namespace octopopros
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

inline std::map<std::string, std::string> paramToArgMappingsOctomap()
{
  std::map<std::string, std::string> param_to_arg =  //
    {
      { "clamp", "clamp" },                    //
      { "collapse", "collapse" },              //
      { "hit", "hit" },                        //
      { "non_lazy", "non-lazy" },              //
      { "miss", "miss" },                      //
      { "offset_origin", "offset-origin" },    //
      { "ray_length_max", "ray-length-max" },  //
      { "threshold", "threshold" }             //
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
void publishMapCloud(ros::Publisher &publisher, const std::string &map_frame, const octomap::OcTree &map,
                     const glm::dvec3 &map_origin, ohmdataros::ColourSolver colour_solver)
{
  if (publisher.getNumSubscribers() == 0)
  {
    return;
  }

  const float resolution = float(map.getResolution());
  sensor_msgs::PointCloud2 map_cloud;
  map_cloud.header.frame_id = map_frame;
  map_cloud.header.stamp = ros::Time::now();

  auto map_iter = map.begin_leafs();
  const auto map_end_iter = map.end_leafs();

  const auto next_point = [&](ohmdataros::Voxel &voxel) {
    while (map_iter != map_end_iter)
    {
      const float occupancy = float(map_iter->getOccupancy());
      const auto coord = map_iter.getCoordinate();
      ++map_iter;

      if (occupancy >= map.getOccupancyThres())
      {
        voxel.x = coord.x() + map_origin.x;
        voxel.y = coord.y() + map_origin.y;
        voxel.z = coord.z() + map_origin.z;
        voxel.intensity = uint16_t(occupancy * float(0xffffu));
        voxel.r = voxel.g = voxel.b = occupancy;
        voxel.scale = resolution;
        const auto colour = colour_solver(voxel);
        voxel.r = colour.rf();
        voxel.g = colour.gf();
        voxel.b = colour.bf();
        voxel.a = colour.af();
        return true;
      }
    }

    return false;
  };

  ohmdataros::populateCloudMessage(map_cloud, next_point);

  publisher.publish(map_cloud);
}


void publishMapVoxels(ros::Publisher &publisher, const std::string &map_frame, const octomap::OcTree &map,
                      const glm::dvec3 &map_origin, const std::string &name, ohmdataros::ColourSolver colour_solver)
{
  if (publisher.getNumSubscribers() == 0)
  {
    return;
  }

  const float resolution = float(map.getResolution());
  visualization_msgs::Marker markers;
  markers.header.frame_id = map_frame;
  markers.header.stamp = ros::Time::now();
  markers.ns = name;
  markers.id = 0;

  auto map_iter = map.begin_leafs();
  const auto map_end_iter = map.end_leafs();

  const auto next_point = [&](ohmdataros::Voxel &voxel) {
    while (map_iter != map_end_iter)
    {
      const float occupancy = float(map_iter->getOccupancy());
      const auto coord = map_iter.getCoordinate();
      ++map_iter;

      if (occupancy >= map.getOccupancyThres())
      {
        voxel.x = coord.x() + map_origin.x;
        voxel.y = coord.y() + map_origin.y;
        voxel.z = coord.z() + map_origin.z;
        voxel.intensity = uint16_t(occupancy * float(0xffffu));
        voxel.r = voxel.g = voxel.b = occupancy;
        voxel.scale = resolution;
        const auto colour = colour_solver(voxel);
        voxel.r = colour.rf();
        voxel.g = colour.gf();
        voxel.b = colour.bf();
        voxel.a = colour.af();
        return true;
      }
    }

    return false;
  };

  // Don't publish empty marker messages. Rviz doesn't like it.
  if (ohmdataros::populateMarkerMessage(markers, next_point) > 0)
  {
    publisher.publish(markers);
  }
}
}  // namespace octopopros

#endif  // OCTOMAPPOPROS_OCTOPOPROS_H

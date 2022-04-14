// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMPOPROS_ROSOHMCLOUD_H
#define OHMPOPROS_ROSOHMCLOUD_H

#include <ohm/MapInfo.h>
#include <ohm/OccupancyMap.h>
#include <ohm/VoxelData.h>

#include <ohmheightmap/Heightmap.h>
#include <ohmheightmap/HeightmapVoxel.h>

#include <ohmutil/Colour.h>

#include <ohmdataros/MapPublish.h>

#include <ros/ros.h>

#include <mutex>
#include <string>


namespace ohmpopros
{
inline constexpr float defaultTsdfSurfaceDistance()
{
  return 0.1f;
}

inline void publishMapCloud(ros::Publisher &publisher, const std::string &map_frame, const ohm::OccupancyMap &map,
                            ohmdataros::ColourSolver colour_solver, std::mutex *publish_mutex = nullptr)
{
  if (publisher.getNumSubscribers() == 0)
  {
    return;
  }

  sensor_msgs::PointCloud2 map_cloud;
  map_cloud.header.frame_id = map_frame;
  map_cloud.header.stamp = ros::Time::now();

  ohm::Voxel<const float> voxel_occupancy(&map, map.layout().occupancyLayer());
  ohm::Voxel<const ohm::VoxelMean> voxel_mean(&map, map.layout().meanLayer());
  ohm::Voxel<const ohm::VoxelTsdf> voxel_tsdf(&map, map.layout().layerIndex(ohm::default_layer::tsdfLayerName()));
  const double map_resolution = map.resolution();
  auto map_iter = map.begin();
  const auto map_end = map.end();

  float surface_distance = static_cast<float>(
    map.mapInfo().get("tsdf-default-truncation-distance", ohm::MapValue("", defaultTsdfSurfaceDistance())));
  surface_distance = std::min(surface_distance, float(0.75 * map_resolution));

  ohmdataros::CloudVoxelFunction next_point;

  if (voxel_occupancy.isLayerValid())
  {
    next_point = [&](ohmdataros::Voxel &voxel) {
      // Loop over points as we look only for occupied voxels.
      while (map_iter != map_end)
      {
        // Set the voxel key
        ohm::setVoxelKey(map_iter, voxel_occupancy, voxel_mean);
        // We can now increment and check for occupancy.
        ++map_iter;
        if (ohm::isOccupied(voxel_occupancy))
        {
          // Occupied voxel. Write data and return.
          const glm::dvec3 position = ohm::positionSafe(voxel_mean);
          const float probability = ohm::valueToProbability(voxel_occupancy.data());
          voxel.x = position.x;
          voxel.y = position.y;
          voxel.z = position.z;
          voxel.scale = map_resolution;
          voxel.intensity = uint16_t(probability * std::numeric_limits<uint16_t>::max());
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
  }
  else if (voxel_tsdf.isLayerValid())
  {
    // TSDF map.
    next_point = [&](ohmdataros::Voxel &voxel) {
      // Loop over points as we look only for occupied voxels.
      while (map_iter != map_end)
      {
        // Set the voxel key
        ohm::setVoxelKey(map_iter, voxel_tsdf);
        // We can now increment and check for occupancy.
        ++map_iter;
        const ohm::VoxelTsdf tsdf = voxel_tsdf.data();
        if (tsdf.weight > 0 && std::abs(tsdf.distance) < surface_distance)
        {
          // Occupied voxel. Write data and return.
          const glm::dvec3 position = voxel_tsdf.map()->voxelCentreGlobal(*map_iter);
          voxel.x = position.x;
          voxel.y = position.y;
          voxel.z = position.z;
          voxel.scale = map_resolution;
          voxel.a = 1.0f;
          voxel.intensity = 0x7fffu;
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
  }
  else
  {
    // No valid layers.
    return;
  }

  ohmdataros::populateCloudMessage(map_cloud, next_point);

  if (publish_mutex)
  {
    std::unique_lock<std::mutex> publish_guard(*publish_mutex);
    publisher.publish(map_cloud);
  }
  else
  {
    publisher.publish(map_cloud);
  }
}

inline void publishMapVoxels(ros::Publisher &publisher, const std::string &map_frame, const ohm::OccupancyMap &map,
                             const std::string &name, ohmdataros::ColourSolver colour_solver,
                             std::mutex *publish_mutex = nullptr)
{
  if (publisher.getNumSubscribers() == 0)
  {
    return;
  }

  visualization_msgs::Marker markers;
  markers.header.frame_id = map_frame;
  markers.header.stamp = ros::Time::now();
  markers.ns = name;
  markers.id = 0;

  // Occupancy based map
  ohm::Voxel<const float> voxel_occupancy(&map, map.layout().occupancyLayer());
  ohm::Voxel<const ohm::VoxelMean> voxel_mean(&map, map.layout().meanLayer());
  ohm::Voxel<const ohm::VoxelTsdf> voxel_tsdf(&map, map.layout().layerIndex(ohm::default_layer::tsdfLayerName()));
  const double map_resolution = map.resolution();
  auto map_iter = map.begin();
  const auto map_end = map.end();

  float surface_distance = static_cast<float>(
    map.mapInfo().get("tsdf-default-truncation-distance", ohm::MapValue("", defaultTsdfSurfaceDistance())));
  surface_distance = std::min(surface_distance, float(0.75 * map_resolution));

  ohmdataros::CloudVoxelFunction next_point;

  if (voxel_occupancy.isLayerValid())
  {
    // Occupancy based map
    next_point = [&](ohmdataros::Voxel &voxel) {
      // Loop over points as we look only for occupied voxels.
      while (map_iter != map_end)
      {
        // Set the voxel key
        ohm::setVoxelKey(map_iter, voxel_occupancy, voxel_mean);
        // We can now increment and check for occupancy.
        ++map_iter;
        if (ohm::isOccupied(voxel_occupancy))
        {
          // Occupied voxel. Write data and return.
          const glm::dvec3 position = ohm::positionSafe(voxel_mean);
          const float probability = ohm::valueToProbability(voxel_occupancy.data());
          voxel.x = position.x;
          voxel.y = position.y;
          voxel.z = position.z;
          voxel.scale = map_resolution;
          voxel.a = 1.0f;
          voxel.intensity = uint16_t(probability * std::numeric_limits<uint16_t>::max());
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
  }
  else if (voxel_tsdf.isLayerValid())
  {
    // TSDF map.
    next_point = [&](ohmdataros::Voxel &voxel) {
      // Loop over points as we look only for occupied voxels.
      while (map_iter != map_end)
      {
        // Set the voxel key
        ohm::setVoxelKey(map_iter, voxel_tsdf);
        // We can now increment and check for occupancy.
        ++map_iter;
        const ohm::VoxelTsdf tsdf = voxel_tsdf.data();
        if (tsdf.weight > 0 && std::abs(tsdf.distance) < surface_distance)
        {
          // Occupied voxel. Write data and return.
          const glm::dvec3 position = voxel_tsdf.map()->voxelCentreGlobal(*map_iter);
          voxel.x = position.x;
          voxel.y = position.y;
          voxel.z = position.z;
          voxel.scale = map_resolution;
          voxel.a = 1.0f;
          voxel.intensity = 0xffffu;
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
  }
  else
  {
    // No valid layers.
    return;
  }

  // Don't publish empty marker messages. Rviz doesn't like it.
  if (ohmdataros::populateMarkerMessage(markers, next_point) > 0)
  {
    if (publish_mutex)
    {
      std::unique_lock<std::mutex> publish_guard(*publish_mutex);
      publisher.publish(markers);
    }
    else
    {
      publisher.publish(markers);
    }
  }
}

/// Generate and publish an heightmap so long the @p publisher has subscribers.
/// @param publisher The marker messages topic publisher.
/// @param map_frame The frame to publish in.
/// @param heightmap The object to generate the heightmap with.
/// @param map The map to generate a heightmap from.
/// @param sensor_position The current sensor position. Used as the seed position for the heightmap.
/// @param name Marker message namespace.
/// @param heightmap_extents Extents of the heightmap to cull to, local to the @p sensor_position.
/// @param publish_mutex Mutex to lock before publishing, if provided.
void publishHeightmap(ros::Publisher &publisher, const std::string &map_frame, ohm::Heightmap &heightmap,
                      const ohm::OccupancyMap &map, const glm::dvec3 &sensor_position,
                      const ohm::Aabb &heightmap_extents, const std::string &name, std::mutex *publish_mutex = nullptr)
{
  if (publisher.getNumSubscribers() == 0)
  {
    // No subscribers. Don't waste time.
    return;
  }

  // Build/update the heightmap.
  heightmap.setOccupancyMap(&map);
  heightmap.buildHeightmap(sensor_position, heightmap_extents + sensor_position);
  heightmap.setOccupancyMap(nullptr);

  visualization_msgs::Marker markers;
  markers.header.frame_id = map_frame;
  markers.header.stamp = ros::Time::now();
  markers.ns = name;
  markers.id = 0;

  // Extract voxels.
  ohm::Voxel<const float> voxel_occupancy(&heightmap.heightmap(), heightmap.heightmap().layout().occupancyLayer());

  if (!voxel_occupancy.isLayerValid())
  {
    return;
  }

  auto map_iter = heightmap.heightmap().begin();
  auto map_end = heightmap.heightmap().end();
  const auto map_resolution = heightmap.heightmap().resolution();

  const ohmdataros::CloudVoxelFunction next_point = [&](ohmdataros::Voxel &voxel) {
    while (map_iter != map_end)
    {
      ohm::setVoxelKey(map_iter, voxel_occupancy);
      ++map_iter;

      if (voxel_occupancy.isValid())
      {
        glm::dvec3 voxel_pos, voxel_min;
        ohm::HeightmapVoxel heightmap_info;
        // Query voxel information.
        auto voxel_type = heightmap.getHeightmapVoxelInfo(voxel_occupancy.key(), &voxel_pos, &heightmap_info);

        // Work out the relevance and colour.
        ohm::Colour voxel_colour{};
        switch (voxel_type)
        {
        default:
        case ohm::HeightmapVoxelType::kUnknown:
        case ohm::HeightmapVoxelType::kVacant:
          // No data or clear for this voxel.
          continue;
        case ohm::HeightmapVoxelType::kSurface:
          // Surface/supporting voxel.
          voxel_colour = ohm::Colour::kColours[ohm::Colour::kLimeGreen];
          break;
        case ohm::HeightmapVoxelType::kVirtualSurface:
          voxel_colour = ohm::Colour::kColours[ohm::Colour::kOrange];
          break;
        case ohm::HeightmapVoxelType::kInferredFatal:
        case ohm::HeightmapVoxelType::kFatal:
          voxel_colour = ohm::Colour::kColours[ohm::Colour::kMagenta];
          break;
        }

        // Add a voxel to the message.
        voxel.x = voxel_pos.x;
        voxel.y = voxel_pos.y;
        voxel.z = voxel_pos.z;
        voxel.scale = map_resolution;
        voxel.r = voxel_colour.rf();
        voxel.g = voxel_colour.gf();
        voxel.b = voxel_colour.bf();
        voxel.a = voxel_colour.af();
        return true;
      }
    }
    return false;
  };

  // Don't publish empty marker messages. Rviz doesn't like it.
  if (ohmdataros::populateMarkerMessage(markers, next_point) > 0)
  {
    if (publish_mutex)
    {
      std::unique_lock<std::mutex> publish_guard(*publish_mutex);
      publisher.publish(markers);
    }
    else
    {
      publisher.publish(markers);
    }
  }
}
}  // namespace ohmpopros

#endif  // OHMPOPROS_ROSOHMCLOUD_H

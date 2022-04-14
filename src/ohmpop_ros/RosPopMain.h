// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMPOPROS_ROSPOPGPU
#define OHMPOPROS_ROSPOPGPU

#include "RosArgMap.h"
#include "RosMap.h"
#include "RosOhmCloud.h"

#include <ohmdataros/RosDataSource.h>

#include <ohmapp/OhmAppGpu.h>
#include <ohmapp/ohmappmain.inl>

#include <ohm/CopyUtil.h>
#include <ohm/DefaultLayer.h>
#include <ohm/OccupancyMap.h>
#include <ohm/VoxelData.h>

#include <ohmheightmap/Heightmap.h>

#include <array>
#include <string>

int ohmpoprosMainGpu(int argc, char **argv, const std::string &name,
                     const std::map<std::string, std::string> &param_to_arg_mappings)
{
  ros::init(argc, argv, name);

  auto data_source = std::make_shared<ohmdataros::RosDataSource>(true);

  // Build command line arguments array.
  std::vector<std::string> args;
  for (int i = 0; i < argc; ++i)
  {
    args.emplace_back(argv[i]);
  }
  // Add arguments from ROS params.
  ohmpopros::migrateParamsToArgs(args, data_source->nodeHandle(), param_to_arg_mappings);
  // And simulate argc/argv
  bool gpu_cache_size_specified = false;
  int argc2 = int(args.size());
  std::vector<const char *> argv2(args.size());
  for (size_t i = 0; i < args.size(); ++i)
  {
    argv2[i] = args[i].c_str();
    gpu_cache_size_specified = gpu_cache_size_specified || args[i].find("--gpu-cache-size") == 0;
  }

  ohmapp::OhmAppGpu populator(data_source);
  populator.options().positional_args = {};

  std::unique_ptr<ohm::Heightmap> heightmap;
  // Heightmap configuration is fixed to keep the example code simple. This defined a very large region for
  // visualisation purposes. Much smaller regions are recommended for real-time usage.
  const double kHeightmapHalfExtents = 10.0;
  const ohm::Aabb local_heightmap_aabb(glm::dvec3(-kHeightmapHalfExtents), glm::dvec3(kHeightmapHalfExtents));

  ros::Publisher cloud_publisher = data_source->nodeHandle().advertise<sensor_msgs::PointCloud2>("cloud", 5);
  ros::Publisher voxel_publisher = data_source->nodeHandle().advertise<visualization_msgs::Marker>("voxels", 5);
  ros::Publisher heightmap_publisher = data_source->nodeHandle().advertise<visualization_msgs::Marker>("heightmap", 5);
  // Shared pointer to the mutext so that we can dangle threads until the finish without destroying the mutex here.
  // They all take a copy of the shared pointer.
  std::shared_ptr<std::mutex> publish_mutex = std::make_shared<std::mutex>();
  std::atomic_bool publish_thread_active(false);

  // Set the default GPU cache size if not specified before we parse the command line. Otherwise it will auto configure.
  if (!gpu_cache_size_specified)
  {
    // Create the map with default region sizing and sub-voxel positioning.
    // Set Gpu layer cache as follows:
    // - We run run the local map at most at a range of 20m^3 @ 10cm and 32x32x32 grid in each region (the default)
    // - Makes a region size ~ 3.2m^2
    // - To cover 20m^3 we need 7x7x7 regions.
    // - Round up to 8x8x8 worst case.
    // - For each region, in GPU we have:
    //    - occupancy layer: 4 bytes per voxel = 128KiB
    //    - voxel average layer: 8 bytes per voxel = 256KiB
    //    - voxel covariance layer: 24 bytes per voxel = 768KiB
    // - Based on the worst case layer, to hold all relevant regions in the cache, we need:
    //    - 8x8x8 * (128 + 256 + 768)KiB cache size => 526MiB
    // - Round up to 1GiB for lots of headroom. Remember this is apportioned across the layers.
    populator.options().gpu().cache_size_gb = 1.0;
  }

  // Setup a MapLayout for syncing back from GPU. We sync to another map, then use a thread to publish.
  // However, we are selective about which layers we sync.
  // Configured in OnStartCallback()
  ohm::MapLayout sync_layout;

  populator.setOnStartCallback([&populator, data_source, &heightmap, &sync_layout]() {
    // Install a ray filter to clip to the bounded region. This is very important for GPU maps to prevent adding bad
    // rays (e.g., NaN data)
    if (auto *map = populator.gpuMap())
    {
      const auto ray_length_max = populator.options().map().ray_length_max;
      map->setRayFilter(
        [data_source, ray_length_max](glm::dvec3 *start, glm::dvec3 *end, unsigned *filter_flags) -> bool {
          return ohmpopros::filterRays(start, end, filter_flags, *data_source, ray_length_max);
        });
    }

    // Setup the sync_layout to sync the layers we need.
    if (populator.map())
    {
      const ohm::MapLayout &src_layout = populator.map()->layout();
      if (src_layout.layerIndex(ohm::default_layer::tsdfLayerName()) >= 0)
      {
        // TSDF map: only need this layer.
        ohm::addTsdf(sync_layout);
      }
      else
      {
        // Need occupancy plus mean if available
        if (src_layout.occupancyLayer() >= 0)
        {
          ohm::addOccupancy(sync_layout);
        }
        if (src_layout.meanLayer() >= 0)
        {
          ohm::addVoxelMean(sync_layout);
        }
      }

      // Setup the heightmap. Heightmap implementation is rough and configurable from the command line.
      const double kMinClearance = 1.0;
      heightmap = std::make_unique<ohm::Heightmap>(populator.map()->resolution(), kMinClearance);
      heightmap->setCeiling(1.5);
      heightmap->setFloor(2.0);
      heightmap->setMinClearance(kMinClearance);
      heightmap->setGenerateVirtualSurface(true);
      heightmap->setPromoteVirtualBelow(true);
      heightmap->setVirtualSurfaceFilterThreshold(3);
      heightmap->setIgnoreVoxelMean(false);
      // Using the simplest heightmap generation for this example.
      heightmap->setMode(ohm::HeightmapMode::kPlanar);
    }
  });

  // Setup map publishing function.
  data_source->setPublishCloudFunction([&populator, data_source, &heightmap, &cloud_publisher, &voxel_publisher,
                                        &heightmap_publisher, local_heightmap_aabb, publish_mutex,
                                        &publish_thread_active, &sync_layout]() {
    const glm::dvec3 sensor_position = data_source->sensorPosition();
    const glm::dvec3 half_extents(data_source->options().map_half_extents, data_source->options().map_half_extents,
                                  (data_source->options().allow_vertical_extents) ?
                                    data_source->options().map_half_extents_vertical :
                                    data_source->options().map_half_extents);
    const glm::dvec3 min_extents = sensor_position - glm::dvec3(half_extents);
    const glm::dvec3 max_extents = sensor_position + glm::dvec3(half_extents);
    if (populator.gpuMap() && populator.map())
    {
      const glm::vec3 colour = populator.options().output().cloud_colour;
      ohmdataros::ColourSolver colour_solver;
      if (glm::all(glm::greaterThanEqual(colour, glm::vec3(0.0f))))
      {
        ohm::Colour colour_int;
        colour_int.setRf(colour.x);
        colour_int.setGf(colour.y);
        colour_int.setBf(colour.z);
        colour_solver = ohmdataros::uniformColourSolver(colour_int);
      }
      else
      {
        colour_solver = ohmdataros::colourByHeight();
      }

      populator.map()->cullRegionsOutside(min_extents, max_extents);
      // Start a publishing thread if not already active and we have subscribers.
      if (!publish_thread_active &&
          (cloud_publisher.getNumSubscribers() > 0 || voxel_publisher.getNumSubscribers() > 0 ||
           heightmap_publisher.getNumSubscribers() > 0))
      {
        // Sync the GPU map to fresh map instance, then push that to another thread for publishing visualisation.
        // This allows the map to continue to be updated while we extract visualisation data.
        // We only need to sync occupancy, mean and covariance layers or just TSDF (depending on the configuration).
        auto map_copy =
          std::make_shared<ohm::OccupancyMap>(populator.map()->resolution(), ohm::MapFlag::kDefault, sync_layout);
        // auto map_copy = std::make_shared<ohm::OccupancyMap>(populator.map()->resolution(), ohm::MapFlag::kDefault,
        //                                                     populator.map()->layout());
        ohm::copyMap(*map_copy, *populator.map());
        publish_thread_active = true;
        std::thread populate_thread([&cloud_publisher, &voxel_publisher, &heightmap_publisher, map_copy, &heightmap,
                                     publish_mutex, sensor_position, local_heightmap_aabb, &publish_thread_active,
                                     map_frame = data_source->options().map_frame, colour_solver] {
          // Mutex guard against multiple publish attempts. Only locked on the final publish calls.
          ohmpopros::publishMapCloud(cloud_publisher, map_frame, *map_copy, ohmdataros::colourByIntensity(),
                                     publish_mutex.get());
          ohmpopros::publishMapVoxels(voxel_publisher, map_frame, *map_copy, "voxels", colour_solver,
                                      publish_mutex.get());
          if (heightmap)
          {
            ohmpopros::publishHeightmap(heightmap_publisher, map_frame, *heightmap, *map_copy, sensor_position,
                                        local_heightmap_aabb, "heightmap", publish_mutex.get());
          }
          publish_thread_active = false;
        });
        // No need to join.
        populate_thread.detach();
      }
    }
  });

  return ohmappMain(argc2, argv2.data(), populator);
}

#endif  // OHMPOPROS_ROSPOPGPU

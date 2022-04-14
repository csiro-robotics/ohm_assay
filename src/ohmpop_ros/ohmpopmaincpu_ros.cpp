
// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include <ohmapp/OhmAppGpu.h>
#include <ohmdataros/RosDataSource.h>
#include <ohmapp/ohmappmain.inl>

#include <ohm/OccupancyMap.h>
#include <ohm/VoxelData.h>

#include "RosArgMap.h"
#include "RosMap.h"
#include "RosOhmCloud.h"

#include <array>
#include <string>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ohmpopcpu_ros");

  auto data_source = std::make_shared<ohmdataros::RosDataSource>(true);

  // Build command line arguments array.
  std::vector<std::string> args;
  for (int i = 0; i < argc; ++i)
  {
    args.emplace_back(argv[i]);
  }
  // Add arguments from ROS params.
  ohmpopros::migrateParamsToArgs(args, data_source->nodeHandle(), ohmpopros::paramToArgMappingsOhmCpu());
  // And simulate argc/argv
  int argc2 = int(args.size());
  std::vector<const char *> argv2(args.size());
  for (size_t i = 0; i < args.size(); ++i)
  {
    argv2[i] = args[i].c_str();
  }

  ohmapp::OhmAppCpu populator(data_source);
  populator.options().positional_args = {};
  ros::Publisher cloud_publisher = data_source->nodeHandle().advertise<sensor_msgs::PointCloud2>("cloud", 5);
  ros::Publisher voxel_publisher = data_source->nodeHandle().advertise<visualization_msgs::Marker>("voxels", 5);

  populator.setOnStartCallback([&populator, data_source]() {
    // Install a ray filter to clip to the bounded region. This is very important for GPU maps to prevent adding bad
    // rays (e.g., NaN data)
    if (auto *map = populator.map())
    {
      const auto ray_length_max = populator.options().map().ray_length_max;
      map->setRayFilter(
        [data_source, ray_length_max](glm::dvec3 *start, glm::dvec3 *end, unsigned *filter_flags) -> bool {
          return ohmpopros::filterRays(start, end, filter_flags, *data_source, ray_length_max);
        });
    }
  });

  // Setup map publishing function.
  data_source->setPublishCloudFunction([&populator, &cloud_publisher, &voxel_publisher, data_source]() {
    const glm::dvec3 sensor_position = data_source->sensorPosition();
    const glm::dvec3 half_extents(data_source->options().map_half_extents, data_source->options().map_half_extents,
                                  (data_source->options().allow_vertical_extents) ?
                                    data_source->options().map_half_extents_vertical :
                                    data_source->options().map_half_extents);
    const glm::dvec3 min_extents = sensor_position - glm::dvec3(half_extents);
    const glm::dvec3 max_extents = sensor_position + glm::dvec3(half_extents);
    if (populator.map())
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

      auto *map = populator.map();
      map->cullRegionsOutside(min_extents, max_extents);
      // Start a publishing thread if not already active and we have subscribers.
      if (cloud_publisher.getNumSubscribers() > 0 || voxel_publisher.getNumSubscribers() > 0)
      {
        const auto map_frame = data_source->options().map_frame;
        ohmpopros::publishMapCloud(cloud_publisher, map_frame, *map, ohmdataros::colourByIntensity());
        ohmpopros::publishMapVoxels(voxel_publisher, map_frame, *map, "voxels", colour_solver);
      }
    }
  });

  return ohmappMain(argc2, argv2.data(), populator);
}

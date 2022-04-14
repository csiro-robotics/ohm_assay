//
// author Kazys Stepanas
//
#include <ohmapp/SlamIOSource.h>
#include <ohmapp/ohmappmain.inl>

#include "OctopopRos.h"

#include <ohmdataros/RosDataSource.h>

#include <octomap/octomap.h>

#include <glm/vector_relational.hpp>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "octopop_ros");

  auto data_source = std::make_shared<ohmdataros::RosDataSource>();

  // Build command line arguments array.
  std::vector<std::string> args;
  for (int i = 0; i < argc; ++i)
  {
    args.emplace_back(argv[i]);
  }
  // Add arguments from ROS params.
  octopopros::migrateParamsToArgs(args, data_source->nodeHandle(), octopopros::paramToArgMappingsOctomap());
  // And simulate argc/argv
  int argc2 = int(args.size());
  std::vector<const char *> argv2(args.size());
  for (size_t i = 0; i < args.size(); ++i)
  {
    argv2[i] = args[i].c_str();
  }

  octomappop::OctomapPop populator(data_source);
  populator.options().positional_args = {};
  ros::Publisher cloud_publisher = data_source->nodeHandle().advertise<sensor_msgs::PointCloud2>("cloud", 5);
  ros::Publisher voxel_publisher = data_source->nodeHandle().advertise<visualization_msgs::Marker>("voxels", 5);

  // Setup map publishing function.
  data_source->setPublishCloudFunction([&populator, &cloud_publisher, &voxel_publisher, data_source]() {
    const glm::dvec3 sensor_position = data_source->sensorPosition();
    const glm::dvec3 min_extents = sensor_position - data_source->options().map_half_extents;
    const glm::dvec3 max_extents = sensor_position + data_source->options().map_half_extents;

    if (populator.map())
    {
      auto &map = *populator.map();
      const auto map_frame = data_source->options().map_frame;
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

      octopopros::publishMapCloud(cloud_publisher, map_frame, map, populator.mapOrigin(),
                                  ohmdataros::colourByIntensity());
      octopopros::publishMapVoxels(voxel_publisher, map_frame, map, populator.mapOrigin(), "voxels", colour_solver);
    }
  });

  return ohmappMain(argc2, argv2.data(), populator);
}

//
// author Kazys Stepanas
//
#include <ohmapp/SlamIOSource.h>
#include <ohmapp/ohmappmain.inl>

#include "VoxbloxPopRos.h"

#include <ohmdataros/RosDataSource.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "voxbloxoccupancy_ros");

  auto data_source = std::make_shared<ohmdataros::RosDataSource>();

  // Build command line arguments array.
  std::vector<std::string> args;
  for (int i = 0; i < argc; ++i)
  {
    args.emplace_back(argv[i]);
  }
  // Add arguments from ROS params.
  voxpopros::migrateParamsToArgs(args, data_source->nodeHandle(), voxpopros::paramToArgMappingsOccupancy());
  // And simulate argc/argv
  int argc2 = int(args.size());
  std::vector<const char *> argv2(args.size());
  for (size_t i = 0; i < args.size(); ++i)
  {
    argv2[i] = args[i].c_str();
  }

  voxbloxpop::VoxbloxPopOccupancy populator(data_source);
  populator.options().positional_args = {};
  ros::Publisher cloud_publisher = data_source->nodeHandle().advertise<sensor_msgs::PointCloud2>("cloud", 5);
  ros::Publisher voxel_publisher = data_source->nodeHandle().advertise<visualization_msgs::Marker>("voxels", 5);

  populator.setOnConfigureOptionsCallback([&populator, data_source]() {
    /// Set the max ray length to match the map half extents if not otherwise set.
    if (populator.options().map().integrator.max_ray_length_m <= 0)
    {
      populator.options().map().integrator.max_ray_length_m = data_source->options().map_half_extents;
    }
  });

  // Setup map publishing function.
  data_source->setPublishCloudFunction([&populator, &cloud_publisher, &voxel_publisher, data_source]() {
    const glm::dvec3 sensor_position = data_source->sensorPosition();
    const glm::dvec3 min_extents = sensor_position - data_source->options().map_half_extents;
    const glm::dvec3 max_extents = sensor_position + data_source->options().map_half_extents;

    if (populator.map() && populator.map()->getOccupancyLayerPtr())
    {
      auto &map = *populator.map();
      const auto map_frame = data_source->options().map_frame;
      const float surface_distance_thresh = map.getOccupancyLayer().voxel_size() * 0.75;
      voxblox::Color preferred_colour(255, 255, 255, 255);
      if (glm::all(glm::greaterThanEqual(populator.options().output().cloud_colour, glm::vec3(0.0f))))
      {
        preferred_colour.r = uint8_t(255.0f * populator.options().output().cloud_colour[0]);
        preferred_colour.g = uint8_t(255.0f * populator.options().output().cloud_colour[1]);
        preferred_colour.b = uint8_t(255.0f * populator.options().output().cloud_colour[2]);
      }

      const voxbloxpop::ShouldVisualizeVoxelColorFunctionType<voxblox::OccupancyVoxel> visualise_voxel =
        [surface_distance_thresh, preferred_colour](const voxblox::OccupancyVoxel &voxel, const voxblox::Point &point,
                                                    voxblox::Color *colour) {
          const float occupancy_threshold = voxblox::logOddsFromProbability(0.5f);
          return voxbloxpop::shouldVisualiseOccupancy(voxel, point, occupancy_threshold, colour, &preferred_colour);
        };

      const glm::vec3 colour = populator.options().output().cloud_colour;
      ohmdataros::ColourSolver colour_solver;
      if (glm::all(glm::greaterThanEqual(colour, glm::vec3(0.0f))))
      {
        colour_solver =
          ohmdataros::uniformColourSolver(ohm::Colour(preferred_colour.r, preferred_colour.g, preferred_colour.b));
      }
      else
      {
        colour_solver = ohmdataros::colourByHeight();
      }

      map.getOccupancyLayerPtr()->removeDistantBlocks(
        voxblox::Point(voxblox::FloatingPoint(sensor_position.x), voxblox::FloatingPoint(sensor_position.y),
                       voxblox::FloatingPoint(sensor_position.z)),
        data_source->options().map_half_extents);
      voxpopros::publishMapCloud(cloud_publisher, map_frame, map.getOccupancyLayer(), visualise_voxel,
                                 ohmdataros::colourByIntensity());
      voxpopros::publishMapVoxels(voxel_publisher, map_frame, map.getOccupancyLayer(), visualise_voxel, "voxels",
                                  colour_solver);
    }
  });

  return ohmappMain(argc2, argv2.data(), populator);
}

// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OctomapPop.h"

#include <ohmapp/DataSource.h>
#include <ohmapp/SlamIOSource.h>

#include <ohmutil/Colour.h>
#include <ohmutil/OhmUtil.h>
#include <ohmutil/PlyPointStream.h>

#include <octomap/octomap.h>

#include <chrono>

// Must be after argument streaming operators.
#include <ohmutil/Options.h>

namespace
{
using Clock = std::chrono::high_resolution_clock;
}  // namespace

namespace octomappop
{
OctomapPop::MapOptions::MapOptions() = default;


void OctomapPop::MapOptions::configure(cxxopts::OptionAdder &adder)
{
  Super::MapOptions::configure(adder);
  // clang-format off
  adder
    ("clamp", "Set probability clamping to the given min/max. Given as a value, not probability.", optVal(prob_range))
    ("collapse", "Collapse the map once fully populated? No effect when --non-lazy is also used.", optVal(collapse))
    ("hit", "The occupancy probability due to a hit. Must be >= 0.5.", optVal(prob_hit))
    ("non-lazy", "Use non-lazy evaluation, collapsing the map on every batch.", optVal(non_lazy_eval))
    ("miss", "The occupancy probability due to a miss. Must be < 0.5.", optVal(prob_miss))
    ("offset-origin", "Offset the map origin by the first sensor position? May expand the range of the map.", optVal(offset_map_origin))
    ("ray-length-max", "Maximum ray length. -1 or 0 for unbounded", optVal(max_ray_length))
    ("threshold", "Sets the occupancy threshold assigned when exporting the map to a cloud.", optVal(prob_thresh)->implicit_value(optStr(prob_thresh)))
    ;
  // clang-format on
}


void OctomapPop::MapOptions::print(std::ostream &out)
{
  Super::MapOptions::print(out);
  out << "Hit probability: " << prob_hit << " (" << octomap::logodds(prob_hit) << ")\n";
  out << "Miss probability: " << prob_miss << " (" << octomap::logodds(prob_miss) << ")\n";
  out << "Probability threshold: " << prob_thresh << '\n';
  out << "Max ray length: " << max_ray_length << '\n';
}


OctomapPop::Options::Options()
{
  map_ = std::make_unique<MapOptions>();
}

OctomapPop::OctomapPop(std::shared_ptr<ohmapp::DataSource> data_source)
  : Super(std::make_unique<Options>(), data_source)
{
  data_source->setSamplesOnly(false);
  data_source->requestBatchSettings(5000, 0.0);
}

OctomapPop::OctomapPop(std::unique_ptr<Options> &&options, std::shared_ptr<ohmapp::DataSource> data_source)
  : Super(std::move(options), data_source)
{
  data_source->setSamplesOnly(false);
  data_source->requestBatchSettings(5000, 0.0);
}


#if SLAMIO_HAVE_PDAL
#define CLOUD_TYPE "PDAL supported point cloud"
#else  // SLAMIO_HAVE_PDAL
#define CLOUD_TYPE "PLY point cloud"
#endif  // SLAMIO_HAVE_PDAL

std::string OctomapPop::description() const
{
  return "Generate an octomap octree from a ray cloud or a point cloud with accompanying "
         "trajectory file. The trajectory marks the scanner trajectory with timestamps "
         "loosely corresponding to cloud point timestamps. Trajectory points are "
         "interpolated for each cloud point based on corresponding times in the "
         "trajectory. A ray cloud uses the normals channel to provide a vector from "
         "point sample back to sensor location (see "
         "https://github.com/csiro-robotics/raycloudtools).\n"
         "\n"
         "The sample file is a " CLOUD_TYPE " file, while the trajectory is either a text "
         "trajectory containing [time x y z <additional>] items per line or is itself a "
         "point cloud file.";
}


int OctomapPop::validateOptions(const cxxopts::ParseResult &parsed)
{
  int return_code = Super::validateOptions(parsed);
  if (return_code)
  {
    return return_code;
  }
  return 0;
}


int OctomapPop::prepareForRun()
{
  map_ = std::make_unique<octomap::OcTree>(options().map().resolution);
  map_->setProbHit(options().map().prob_hit);
  map_->setOccupancyThres(options().map().prob_thresh);
  map_->setProbMiss(options().map().prob_miss);
  if (options().map().prob_range[0] > 0)
  {
    map_->setClampingThresMin(options().map().prob_range[0]);
  }
  if (options().map().prob_range[1] > 0)
  {
    map_->setClampingThresMax(options().map().prob_range[1]);
  }
  return 0;
}


bool OctomapPop::processBatch(const glm::dvec3 &batch_origin, const std::vector<glm::dvec3> &sensor_and_samples,
                              const std::vector<double> &timestamps, const std::vector<float> &intensities,
                              const std::vector<glm::vec4> &colours)
{
  // Unused
  (void)batch_origin;
  (void)timestamps;
  (void)intensities;
  (void)colours;

  const MapOptions &map_options = options().map();
  if (!have_updated_ && !timestamps.empty())
  {
    if (map_options.offset_map_origin)
    {
      // Octomap is limited in the volume it can address. We can easily exceed this at high voxel densities. We offset
      // the entire map by taking the first point as the origin of the map to try improve the range.
      map_origin_ = batch_origin;
    }
    have_updated_ = true;
  }

  // Note: we do not use octomap's batch ray integration for two reasons:
  // 1. This allows us to use individual sensor positions for each sample and we can apply the map origin.
  // 2. It tries to use OpenMP, but in a way which is empirically slower than using a single thread.
  const bool lazy_eval = !options().map().non_lazy_eval;
  const double max_range = (map_options.max_ray_length) ? map_options.max_ray_length : -1.0;
  for (size_t i = 0; i < sensor_and_samples.size(); i += 2)
  {
    const auto sensor = sensor_and_samples[i + 0] - map_origin_;
    const auto sample = sensor_and_samples[i + 1] - map_origin_;
    map_->insertRay(octomap::point3d{ float(sensor.x), float(sensor.y), float(sensor.z) },
                    octomap::point3d{ float(sample.x), float(sample.y), float(sample.z) }, max_range, lazy_eval);
  }
  progress_.incrementProgressBy(timestamps.size());
  return !quitPopulation();
}


void OctomapPop::finaliseMap()
{
  if (!options().map().non_lazy_eval && options().map().collapse)
  {
    ohm::logger::info("Collapsing map\n");
    const auto collapse_start_time = Clock::now();
    map_->updateInnerOccupancy();
    const auto collapse_end_time = Clock::now();
    ohm::logger::info("Collapsed time: ", (collapse_end_time - collapse_start_time), '\n');
  }
}


int OctomapPop::saveMap(const std::string &path_without_extension)
{
  const std::string output_file = path_without_extension + ".bt";
  ohm::logger::info("Saving map to ", output_file.c_str(), '\n');

  bool ok = map_->writeBinary(output_file);
  if (!ok)
  {
    ohm::logger::error("Failed to save map\n");
    return -1;
  }
  return 0;
}


int OctomapPop::saveCloud(const std::string &path_ply)
{
  // Save a cloud representation. Need to walk the tree leaves.
  ohm::logger::info("Converting to point cloud ", path_ply, '\n');
  ohm::PlyPointStream ply({ ohm::PlyPointStream::Property{ "x", ohm::PlyPointStream::Type::kFloat64 },
                            ohm::PlyPointStream::Property{ "y", ohm::PlyPointStream::Type::kFloat64 },
                            ohm::PlyPointStream::Property{ "z", ohm::PlyPointStream::Type::kFloat64 },
                            ohm::PlyPointStream::Property{ "red", ohm::PlyPointStream::Type::kUInt8 },
                            ohm::PlyPointStream::Property{ "green", ohm::PlyPointStream::Type::kUInt8 },
                            ohm::PlyPointStream::Property{ "blue", ohm::PlyPointStream::Type::kUInt8 } });
  std::ofstream fout(path_ply, std::ios::binary);
  ply.open(fout);

  const bool occupancy_colour = glm::all(glm::greaterThanEqual(options().output().cloud_colour, glm::vec3(0.0f)));
  const ohm::Colour c = ohm::Colour::fromRgbf(options().output().cloud_colour.r, options().output().cloud_colour.g,
                                              options().output().cloud_colour.b);

  const auto colour_by_occupancy = [](float occupancy) {
    ohm::Colour colour = ohm::Colour::kColours[ohm::Colour::kLightSteelBlue];
    // Occupancy will be at least the occupancy threshold (normaly 50%) so this will only darken to that level.
    colour.adjust(occupancy);
    return colour;
  };

  const auto map_end_iter = map_->end_leafs();
  for (auto iter = map_->begin_leafs(); iter != map_end_iter && quitLevel() < 2; ++iter)
  {
    const float occupancy = float(iter->getOccupancy());
    if (occupancy >= map_->getOccupancyThres())
    {
      const auto coord = iter.getCoordinate();
      const auto point = glm::dvec3(coord.x(), coord.y(), coord.z()) + mapOrigin();

      const ohm::Colour point_colour = (occupancy_colour) ? c : colour_by_occupancy(occupancy);

      ply.setPointPosition(point);
      ply.setProperty("red", point_colour.r());
      ply.setProperty("green", point_colour.g());
      ply.setProperty("blue", point_colour.b());
      ply.writePoint();
    }
  }

  ply.close();
  ohm::logger::info(ply.pointCount(), " point(s) saved", '\n');
  return 0;
}


void OctomapPop::tearDown()
{
  map_.release();
}
}  // namespace octomappop

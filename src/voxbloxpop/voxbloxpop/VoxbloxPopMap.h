// Author: Kazys Stepanas
// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef VOXBLOXPOPMAP_H_
#define VOXBLOXPOPMAP_H_

#include <voxblox/core/common.h>
#include <voxblox/core/occupancy_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/occupancy_integrator.h>
#include <voxblox/integrator/tsdf_integrator.h>

#include <ohmutil/Colour.h>
#include <ohmutil/PlyPointStream.h>

#include <ohmapp/DataSource.h>
#include <ohmapp/MapHarness.h>

#include <glm/vec4.hpp>
#include <glm/vector_relational.hpp>

#include <fstream>
#include <memory>
#include <sstream>

// Delcare streaming operators for argument parsing.

std::istream &operator>>(std::istream &in, voxblox::TsdfIntegratorType &type);
std::ostream &operator<<(std::ostream &out, voxblox::TsdfIntegratorType type);

namespace voxbloxpop
{
/// Population harness to generate an @c ohm::OccupancyMap using ohm CPU algorithms.
template <typename VoxMap, typename VoxIntegrator, typename VoxelType, typename VoxMapOptions>
class VoxbloxPopMap : public ohmapp::MapHarness
{
public:
  /// Base class alias.
  using Super = ohmapp::MapHarness;

  /// Allow long rays by default. Used if not otherwise specified or overridden.
  static inline constexpr double defaultMaxRayLength() { return 200; }

  /// Specialise collated options.
  struct Options : public Super::Options
  {
    inline VoxMapOptions &map() { return static_cast<VoxMapOptions &>(*map_); }
    inline const VoxMapOptions &map() const { return static_cast<const VoxMapOptions &>(*map_); }

    Options();
  };

  /// Default constructor.
  VoxbloxPopMap(std::shared_ptr<ohmapp::DataSource> data_source);

  std::string description() const override;

  const Options &options() const { return static_cast<const Options &>(Super::options()); }

  Options &options() { return static_cast<Options &>(Super::options()); }

  VoxMap *map() { return map_.get(); }
  const VoxMap *map() const { return map_.get(); }

protected:
  VoxbloxPopMap(std::unique_ptr<Options> &&options, std::shared_ptr<ohmapp::DataSource> data_source);

  int validateOptions(const cxxopts::ParseResult &parsed) override;
  int prepareForRun() override;
  bool processBatch(const glm::dvec3 &batch_origin, const std::vector<glm::dvec3> &sensor_and_samples,
                    const std::vector<double> &timestamps, const std::vector<float> &intensities,
                    const std::vector<glm::vec4> &colours) override;
  int saveMap(const std::string &path_without_extension) override;
  int saveCloud(const std::string &path_ply) override;
  void tearDown() override;

  std::unique_ptr<VoxMap> map_;
  std::unique_ptr<VoxIntegrator> integrator_;
  voxblox::Pointcloud points_;
  voxblox::Colors colours_;
};

struct VoxbloxOccupancyMapOptions : public ohmapp::MapHarness::MapOptions
{
  /// Base class alias.
  using Super = ohmapp::MapHarness;

  voxblox::OccupancyMap::Config map{};
  voxblox::OccupancyIntegrator::Config integrator{};

  VoxbloxOccupancyMapOptions();

  int validate();
  void configure(cxxopts::OptionAdder &adder) override;
  void print(std::ostream &out) override;
};


struct VoxbloxTsdfMapOptions : public ohmapp::MapHarness::MapOptions
{
  /// Base class alias.
  using Super = ohmapp::MapHarness;

  voxblox::TsdfMap::Config map{};
  voxblox::TsdfIntegratorBase::Config integrator{};
  voxblox::TsdfIntegratorType mode = voxblox::TsdfIntegratorType::kFast;
  /// Distance at which a voxel is considered a surface voxel.
  ///
  /// Some oddities here. The voxblox code uses 0.75 * voxel_size which seems reasonable, but we only see good behaviour
  /// for this when the voxel size is 0.1. Maybe it's bound to point density? At any setting a configurable value is
  // the easy out here, defaulting to a value which seems to "work".
  float surface_distance_threshold = 0.075f;

  VoxbloxTsdfMapOptions();

  int validate();
  void configure(cxxopts::OptionAdder &adder) override;
  void print(std::ostream &out) override;
};

int createMap(std::unique_ptr<voxblox::OccupancyMap> &map, std::unique_ptr<voxblox::OccupancyIntegrator> &integrator,
              const VoxbloxOccupancyMapOptions &options);

int createMap(std::unique_ptr<voxblox::TsdfMap> &map, std::unique_ptr<voxblox::TsdfIntegratorBase> &integrator,
              const VoxbloxTsdfMapOptions &options);


bool shouldVisualiseTsdf(const voxblox::TsdfVoxel &voxel, const voxblox::Point &point,
                         const float surface_distance_thresh, voxblox::Color *colour,
                         const voxblox::Color *preferred_colour = nullptr);


bool shouldVisualiseOccupancy(const voxblox::OccupancyVoxel &voxel, const voxblox::Point &point,
                              const float occupancy_threshold, voxblox::Color *colour,
                              const voxblox::Color *preferred_colour = nullptr);


void saveVoxbloxCloud(ohm::PlyPointStream &ply, const voxblox::TsdfMap &map, const ohm::Colour &cloud_colour);
void saveVoxbloxCloud(ohm::PlyPointStream &ply, const voxblox::OccupancyMap &map, const ohm::Colour &cloud_colour);

void saveVoxbloxMap(const std::string &filename, const voxblox::TsdfMap &map, const voxblox::TsdfMap::Config &config);
void saveVoxbloxMap(const std::string &filename, const voxblox::OccupancyMap &map,
                    const voxblox::OccupancyMap::Config &config);

#define VPH_TEMPLATE typename VoxMap, typename VoxIntegrator, typename VoxelType, typename VoxMapOptions
#define VPH_TARGS    VoxMap, VoxIntegrator, VoxelType, VoxMapOptions


template <VPH_TEMPLATE>
VoxbloxPopMap<VPH_TARGS>::Options::Options()
{
  map_ = std::make_unique<VoxMapOptions>();
}


template <VPH_TEMPLATE>
VoxbloxPopMap<VPH_TARGS>::VoxbloxPopMap(std::shared_ptr<ohmapp::DataSource> data_source)
  : Super(std::make_unique<Options>(), data_source)
{
  // voxblox assumes one sensor transform for a batch of points.
  data_source->setSamplesOnly(true);
  data_source->requestBatchSettings(5000, 0.1);
}


template <VPH_TEMPLATE>
VoxbloxPopMap<VPH_TARGS>::VoxbloxPopMap(std::unique_ptr<Options> &&options,
                                        std::shared_ptr<ohmapp::DataSource> data_source)
  : Super(std::move(options), data_source)
{
  // voxblox assumes one sensor transform for a batch of points.
  data_source->setSamplesOnly(true);
  data_source->requestBatchSettings(5000, 0.1);
}


template <VPH_TEMPLATE>
std::string VoxbloxPopMap<VPH_TARGS>::description() const
{
  return "Generate a voxblox map.";
}


template <VPH_TEMPLATE>
int VoxbloxPopMap<VPH_TARGS>::validateOptions(const cxxopts::ParseResult &parsed)
{
  if (options().map().integrator.max_ray_length_m <= 0)
  {
    options().map().integrator.max_ray_length_m = defaultMaxRayLength();
  }

  int return_code = Super::validateOptions(parsed);
  if (return_code)
  {
    return return_code;
  }
  VoxMapOptions &map_options = options().map();
  return map_options.validate();
}


template <VPH_TEMPLATE>
int VoxbloxPopMap<VPH_TARGS>::prepareForRun()
{
  return createMap(map_, integrator_, options().map());
}


inline void integrateBatch(voxblox::TsdfIntegratorBase &integrator, const voxblox::Pointcloud &points,
                           const voxblox::Colors &colours, const voxblox::Transformation &sensor_transform)
{
  const bool is_freespace_pointcloud = false;
  integrator.integratePointCloud(sensor_transform, points, colours, is_freespace_pointcloud);
}


inline void integrateBatch(voxblox::OccupancyIntegrator &integrator, const voxblox::Pointcloud &points,
                           const voxblox::Colors &colours, const voxblox::Transformation &sensor_transform)
{
  (void)colours;
  integrator.integratePointCloud(sensor_transform, points);
}


template <VPH_TEMPLATE>
bool VoxbloxPopMap<VPH_TARGS>::processBatch(const glm::dvec3 &batch_origin, const std::vector<glm::dvec3> &samples,
                                            const std::vector<double> &timestamps,
                                            const std::vector<float> &intensities,
                                            const std::vector<glm::vec4> &colours)
{
  // We only need to set the sensor position as our points are global.
  voxblox::Transformation sensor_transform;
  sensor_transform.getPosition() =
    voxblox::Transformation::Position(voxblox::FloatingPoint(batch_origin.x), voxblox::FloatingPoint(batch_origin.y),
                                      voxblox::FloatingPoint(batch_origin.z));

  points_.clear();
  colours_.clear();

  points_.reserve(samples.size());
  colours_.reserve(colours.size());

  // Prepare the @p points by subtracting the batch_origin position.
  // In this context, we assume points and sensor are both in a global frame. However, voxblox requires points in a
  // local frame, so we transform them into sensor space assuming sensor_transform rotation is identity.
  for (const auto &sample : samples)
  {
    const auto pt = sample - batch_origin;
    points_.emplace_back(
      voxblox::Point(voxblox::FloatingPoint(pt.x), voxblox::FloatingPoint(pt.y), voxblox::FloatingPoint(pt.z)));
  }

  for (const auto &colour : colours)
  {
    colours_.emplace_back(voxblox::Color(colour.x, colour.y, colour.z));
  }

  integrateBatch(*integrator_, points_, colours_, sensor_transform);
  progress_.incrementProgressBy(timestamps.size());

  return !quitPopulation();
}


template <VPH_TEMPLATE>
int VoxbloxPopMap<VPH_TARGS>::saveMap(const std::string &path_without_extension)
{
  // Not supported.
  const std::string filename = path_without_extension + ".ohm";

  // Save a cloud representation. Need to walk the tree leaves.
  ohm::logger::info("Converting to point cloud ", path_without_extension, '\n');

  saveVoxbloxMap(filename, *map_, options().map().map);

  return 0;
}


template <typename VoxelType>
using ShouldVisualizeVoxelColorFunctionType =
  std::function<bool(const VoxelType &voxel, const voxblox::Point &coord, voxblox::Color *colour)>;


// From voxblox_ros: ptcloud_vis.h
template <typename VoxelType>
void createPointcloudFromLayer(const voxblox::Layer<VoxelType> &layer,
                               ShouldVisualizeVoxelColorFunctionType<VoxelType> vis_function, ohm::PlyPointStream &ply)
{
  voxblox::BlockIndexList blocks;
  layer.getAllAllocatedBlocks(&blocks);

  // Cache layer settings.
  size_t vps = layer.voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  // Temp variables.
  voxblox::Color color;
  // Iterate over all blocks.
  for (const voxblox::BlockIndex &index : blocks)
  {
    // Iterate over all voxels in said blocks.
    const voxblox::Block<VoxelType> &block = layer.getBlockByIndex(index);

    for (size_t linear_index = 0; linear_index < num_voxels_per_block; ++linear_index)
    {
      voxblox::Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
      const auto &voxel = block.getVoxelByLinearIndex(linear_index);
      if (vis_function(voxel, coord, &color))
      {
        const glm::dvec3 point(coord.x(), coord.y(), coord.z());
        ply.setPointPosition(point);
        ply.setProperty("red", color.r);
        ply.setProperty("green", color.g);
        ply.setProperty("blue", color.b);
        ply.writePoint();
      }
    }
  }
}


template <VPH_TEMPLATE>
int VoxbloxPopMap<VPH_TARGS>::saveCloud(const std::string &path_ply)
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
  ohm::Colour cloud_colour;
  if (glm::all(glm::greaterThanEqual(options().output().cloud_colour, glm::vec3(0.0f))))
  {
    cloud_colour = ohm::Colour(uint8_t(255.0f * options().output().cloud_colour[0]),
                               uint8_t(255.0f * options().output().cloud_colour[1]),
                               uint8_t(255.0f * options().output().cloud_colour[2]));
  }
  else
  {
    cloud_colour = ohm::Colour(255, 255, 255);
  }
  saveVoxbloxCloud(ply, *map_, cloud_colour);
  ply.close();
  ohm::logger::info(ply.pointCount(), " point(s) saved", '\n');
}


template <VPH_TEMPLATE>
void VoxbloxPopMap<VPH_TARGS>::tearDown()
{
  integrator_.reset();
  map_.reset();
}


using VoxbloxPopOccupancy = VoxbloxPopMap<voxblox::OccupancyMap, voxblox::OccupancyIntegrator, voxblox::OccupancyVoxel,
                                          VoxbloxOccupancyMapOptions>;
using VoxbloxPopTsdf =
  VoxbloxPopMap<voxblox::TsdfMap, voxblox::TsdfIntegratorBase, voxblox::TsdfVoxel, VoxbloxTsdfMapOptions>;
}  // namespace voxbloxpop

#undef VPH_TEMPLATE
#undef VPH_TARGS

#endif  // VOXBLOXPOPMAP_H_

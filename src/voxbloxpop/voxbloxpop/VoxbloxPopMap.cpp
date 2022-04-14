// Author: Kazys Stepanas
// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "VoxbloxPopMap.h"

#include <ohm/DefaultLayer.h>
#include <ohm/MapSerialise.h>
#include <ohm/OccupancyMap.h>
#include <ohm/VoxelData.h>

// Must be included after declaring any streaming operators for argument parsing.
#include <ohmutil/Options.h>

namespace
{
/// Enum offset added to the name index to resolve the TsdfIntegratorType
inline constexpr int tsdfTypeOffset()
{
  return 1;
}
}  // namespace

std::istream &operator>>(std::istream &in, voxblox::TsdfIntegratorType &type)
{
  std::string type_str;
  in >> type_str;
  for (int i = 0; i < int(voxblox::kTsdfIntegratorTypeNames.size()); ++i)
  {
    if (type_str == voxblox::kTsdfIntegratorTypeNames[i])
    {
      type = voxblox::TsdfIntegratorType(i + tsdfTypeOffset());
      return in;
    }
  }

  throw cxxopts::invalid_option_format_error(type_str);
}


std::ostream &operator<<(std::ostream &out, voxblox::TsdfIntegratorType type)
{
  out << voxblox::kTsdfIntegratorTypeNames[int(type) - tsdfTypeOffset()];
  return out;
}


namespace voxbloxpop
{
VoxbloxOccupancyMapOptions::VoxbloxOccupancyMapOptions()
{
  map.occupancy_voxel_size = resolution;
  integrator.probability_hit = 0.9f;
  integrator.probability_miss = 0.45f;
  // integrator.threshold_min = 0;
  // integrator.threshold_max = 0;
  integrator.threshold_occupancy = 0.5f;
  integrator.min_ray_length_m = 0;
  integrator.max_ray_length_m = 0;
  // integrator.max_consecutive_ray_collisions = 2;
}

int VoxbloxOccupancyMapOptions::validate()
{
  // Copy map resolution to map config.
  map.occupancy_voxel_size = resolution;
  return 0;
}

void VoxbloxOccupancyMapOptions::configure(cxxopts::OptionAdder &adder)
{
  Super::MapOptions::configure(adder);
  // clang-format off
    adder
      ("dim", "Set the voxel dimensions of each region in the map. Range for each is [0, 255).", optVal(map.occupancy_voxels_per_side))
      ("hit", "The occupancy probability due to a hit. Must be >= 0.5.", optVal(integrator.probability_hit))
      ("miss", "The occupancy probability due to a miss. Must be < 0.5.", optVal(integrator.probability_miss))
      ("ray-length-min", "Minimum ray length.", optVal(integrator.min_ray_length_m))
      ("ray-length-max", "Maximum ray length. 0 to use default", optVal(integrator.max_ray_length_m))
      ("threshold", "Sets the occupancy threshold assigned when exporting the map to a cloud.", optVal(integrator.threshold_occupancy)->implicit_value(optStr(integrator.threshold_occupancy)))
      ;
  // clang-format on
}

void VoxbloxOccupancyMapOptions::print(std::ostream &out)
{
  Super::MapOptions::print(out);
  out << "Voxels per side: " << map.occupancy_voxels_per_side << '\n';
  out << "Hit probability: " << integrator.probability_hit << "\n";
  out << "Miss probability: " << integrator.probability_miss << "\n";
  out << "Probability threshold: " << integrator.threshold_occupancy << '\n';
}


VoxbloxTsdfMapOptions::VoxbloxTsdfMapOptions()
{
  map.tsdf_voxel_size = resolution;
  integrator.voxel_carving_enabled = true;
  integrator.min_ray_length_m = 0;
  integrator.max_ray_length_m = 0;  // Will use default on start.
  integrator.use_const_weight = true;
  integrator.allow_clear = true;
  integrator.use_weight_dropoff = true;
  // Assume sparse clouds (lidar data not depth camera)
  integrator.use_sparsity_compensation_factor = true;
  integrator.sparsity_compensation_factor = 10.0f;
  // Benefits drop off at 6 threads. E.g., second thread is just less than 50% faster, four is less than 50% on top.
  integrator.integrator_threads = std::min<size_t>(integrator.integrator_threads, 6u);
  integrator.integration_order_mode = "sorted";
  integrator.enable_anti_grazing = false;
}

int VoxbloxTsdfMapOptions::validate()
{
  // Copy map resolution to map config.
  map.tsdf_voxel_size = resolution;
  return 0;
}

void VoxbloxTsdfMapOptions::configure(cxxopts::OptionAdder &adder)
{
  Super::MapOptions::configure(adder);
  // clang-format off
    adder
      ("allow-clear", "Allow clearing?", optVal(integrator.allow_clear))
      ("anti-grazing", "Enable anti grazing? (merge mode)", optVal(integrator.enable_anti_grazing))
      ("dim", "Set the voxel dimensions of each region in the map. Range for each is [0, 255).", optVal(map.tsdf_voxels_per_side))
      ("max-consecutive-ray-collisions", "Maximum ray collisions for a voxel before aborting further updates on that voxel in the current ray set.", optVal(integrator.max_consecutive_ray_collisions))
      ("max-weight", "Maximum integrator weight.", optVal(integrator.max_weight))
      ("mode", "TSDF integrator mode [simple, merged, fast].", optVal(mode))
      ("ray-length-min", "Minimum ray length.", optVal(integrator.min_ray_length_m))
      ("ray-length-max", "Maximum ray length. 0 to use default.", optVal(integrator.max_ray_length_m))
      ("sparsity-compensation", "Sparsity compensation factor.", optVal(integrator.sparsity_compensation_factor))
      ("surface-threshold", "Surface distance threshold used in export/visualisation.", optVal(surface_distance_threshold))
      ("threads", "Number of integrator threads to use.", optVal(integrator.integrator_threads))
      ("trunc-dist", "Default truncation distance.", optVal(integrator.default_truncation_distance))
      ("use-const-weight", "Use constant weight?", optVal(integrator.use_const_weight))
      ("use-weight-dropoff", "Use weight dropoff?", optVal(integrator.use_weight_dropoff))
      ("use-sparsity-compensation", "Use the sparsity-comensation?.", optVal(integrator.use_sparsity_compensation_factor))
      ("voxel-carving", "Enable voxel carving?", optVal(integrator.voxel_carving_enabled))
      ;
  // clang-format on
}
void VoxbloxTsdfMapOptions::print(std::ostream &out)
{
  Super::MapOptions::print(out);
  out << "Voxels per side: " << map.tsdf_voxels_per_side << '\n';
  out << "TSDF mode: " << mode << '\n';
  out << "Default truncation distance: " << integrator.default_truncation_distance << '\n';
  out << "Max weight: " << integrator.max_weight << '\n';
  out << "Minimum ray length: " << integrator.min_ray_length_m << '\n';
  out << "Maximum ray length: " << integrator.max_ray_length_m << '\n';
  out << "Maximum consecutive ray collisions: " << integrator.max_consecutive_ray_collisions << '\n';
  out << "Use sparsity compensation: " << integrator.use_sparsity_compensation_factor << '\n';
  if (integrator.use_sparsity_compensation_factor)
  {
    out << "Sparsity compensation factor: " << integrator.sparsity_compensation_factor << '\n';
  }
  out << "Voxel carving: " << integrator.voxel_carving_enabled << '\n';
  out << "Use constant weigth: " << integrator.use_const_weight << '\n';
  out << "Allow clear: " << integrator.allow_clear << '\n';
  out << "Use weight dropoff: " << integrator.use_weight_dropoff << '\n';
  out << "Surface distance threshold: " << surface_distance_threshold << '\n';
  out << "Threads: " << integrator.integrator_threads << '\n';
}


int createMap(std::unique_ptr<voxblox::OccupancyMap> &map, std::unique_ptr<voxblox::OccupancyIntegrator> &integrator,
              const VoxbloxOccupancyMapOptions &options)
{
  map = std::make_unique<voxblox::OccupancyMap>(options.map);
  integrator = std::make_unique<voxblox::OccupancyIntegrator>(options.integrator, map->getOccupancyLayerPtr());
  return 0;
}

int createMap(std::unique_ptr<voxblox::TsdfMap> &map, std::unique_ptr<voxblox::TsdfIntegratorBase> &integrator,
              const VoxbloxTsdfMapOptions &options)
{
  map = std::make_unique<voxblox::TsdfMap>(options.map);

  switch (options.mode)
  {
  default:
  case voxblox::TsdfIntegratorType::kSimple:
    integrator = std::make_unique<voxblox::SimpleTsdfIntegrator>(options.integrator, map->getTsdfLayerPtr());
    break;
  case voxblox::TsdfIntegratorType::kMerged:
    integrator = std::make_unique<voxblox::MergedTsdfIntegrator>(options.integrator, map->getTsdfLayerPtr());
    break;
  case voxblox::TsdfIntegratorType::kFast:
    integrator = std::make_unique<voxblox::FastTsdfIntegrator>(options.integrator, map->getTsdfLayerPtr());
    break;
  }
  return 0;
}

bool shouldVisualiseTsdf(const voxblox::TsdfVoxel &voxel, const voxblox::Point &point,
                         const float surface_distance_thresh, voxblox::Color *colour,
                         const voxblox::Color *preferred_colour)
{
  constexpr float kMinWeight = 0;
  if (voxel.weight > kMinWeight && std::abs(voxel.distance) < surface_distance_thresh)
  {
    if (preferred_colour && (preferred_colour->r != 0 || preferred_colour->g != 0 || preferred_colour->b != 0))
    {
      colour->r = preferred_colour->r;
      colour->g = preferred_colour->g;
      colour->b = preferred_colour->b;
      colour->a = preferred_colour->a;
    }
    else
    {
      *colour = voxel.color;
    }
    return true;
  }
  return false;
}


bool shouldVisualiseOccupancy(const voxblox::OccupancyVoxel &voxel, const voxblox::Point &point,
                              const float occupancy_threshold, voxblox::Color *colour,
                              const voxblox::Color *preferred_colour)
{
  if (voxel.observed && voxel.probability_log >= occupancy_threshold)
  {
    if (preferred_colour && (preferred_colour->r != 0 || preferred_colour->g != 0 || preferred_colour->b != 0))
    {
      colour->r = preferred_colour->r;
      colour->g = preferred_colour->g;
      colour->b = preferred_colour->b;
      colour->a = preferred_colour->a;
    }
    else
    {
      const float voxel_probability = voxblox::probabilityFromLogOdds(voxel.probability_log);
      colour->r = uint8_t(voxel_probability * 255.0f);
      colour->g = uint8_t(voxel_probability * 255.0f);
      colour->b = uint8_t(voxel_probability * 255.0f);
      colour->a = 255u;
    }
    return true;
  }
  return false;
}


void saveVoxbloxCloud(ohm::PlyPointStream &ply, const voxblox::TsdfMap &map, const ohm::Colour &cloud_colour)
{
  const float surface_distance_thresh = map.getTsdfLayer().voxel_size() * 0.75;
  // const auto visualise_voxel =
  const ShouldVisualizeVoxelColorFunctionType<voxblox::TsdfVoxel> visualise_voxel =
    [surface_distance_thresh, cloud_colour](const voxblox::TsdfVoxel &voxel, const voxblox::Point &point,
                                            voxblox::Color *colour) {
      voxblox::Color preferred_colour;
      preferred_colour.r = cloud_colour.r();
      preferred_colour.g = cloud_colour.g();
      preferred_colour.b = cloud_colour.b();
      preferred_colour.a = 255u;
      return shouldVisualiseTsdf(voxel, point, surface_distance_thresh, colour, &preferred_colour);
    };
  createPointcloudFromLayer(map.getTsdfLayer(), visualise_voxel, ply);
}

void saveVoxbloxCloud(ohm::PlyPointStream &ply, const voxblox::OccupancyMap &map, const ohm::Colour &cloud_colour)
{
  const float occupancy_threshold = voxblox::logOddsFromProbability(0.5f);
  // const auto visualise_voxel =
  const ShouldVisualizeVoxelColorFunctionType<voxblox::OccupancyVoxel> visualise_voxel =
    [occupancy_threshold, cloud_colour](const voxblox::OccupancyVoxel &voxel, const voxblox::Point &point,
                                        voxblox::Color *colour) {
      voxblox::Color preferred_colour;
      preferred_colour.r = cloud_colour.r();
      preferred_colour.g = cloud_colour.g();
      preferred_colour.b = cloud_colour.b();
      preferred_colour.a = 255u;
      return shouldVisualiseOccupancy(voxel, point, occupancy_threshold, colour, &preferred_colour);
    };
  createPointcloudFromLayer(map.getOccupancyLayer(), visualise_voxel, ply);
}


void saveVoxbloxMap(const std::string &filename, const voxblox::TsdfMap &map, const voxblox::TsdfMap::Config &config)
{
  // Convert to ohm format and save.
  ohm::MapLayout layout;
  ohm::addTsdf(layout);
  glm::u8vec3 dim(config.tsdf_voxels_per_side);
  ohm::OccupancyMap ohm_map(map.voxel_size(), dim, ohm::MapFlag::kNone, layout);

  const auto &layer = map.getTsdfLayer();

  voxblox::BlockIndexList blocks;
  layer.getAllAllocatedBlocks(&blocks);

  // Cache layer settings.
  size_t vps = layer.voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  ohm::Voxel<ohm::VoxelTsdf> ohm_voxel(&ohm_map, ohm_map.layout().layerIndex(ohm::default_layer::tsdfLayerName()));
  if (!ohm_voxel.isLayerValid())
  {
    // Shouldn't really happen.
    return;
  }

  // Iterate over all blocks.
  for (const voxblox::BlockIndex &index : blocks)
  {
    // Iterate over all voxels in said blocks.
    const voxblox::Block<voxblox::TsdfVoxel> &block = layer.getBlockByIndex(index);

    for (size_t linear_index = 0; linear_index < num_voxels_per_block; ++linear_index)
    {
      voxblox::Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
      const auto &voxel = block.getVoxelByLinearIndex(linear_index);

      const ohm::Key ohm_key = ohm_map.voxelKey(glm::dvec3(coord.x(), coord.y(), coord.z()));
      ohm::setVoxelKey(ohm_key, ohm_voxel);

      if (ohm_voxel.isValid())
      {
        auto voxel_data = ohm_voxel.data();
        voxel_data.distance = voxel.distance;
        voxel_data.weight = voxel.weight;
        ohm_voxel.write(voxel_data);
      }
    }
  }

  ohm::save(filename.c_str(), ohm_map);
}


void saveVoxbloxMap(const std::string &filename, const voxblox::OccupancyMap &map,
                    const voxblox::OccupancyMap::Config &config)
{
  // Convert to ohm format and save.
  glm::u8vec3 dim(config.occupancy_voxels_per_side);
  ohm::OccupancyMap ohm_map(config.occupancy_voxel_size, dim);

  const auto &layer = map.getOccupancyLayer();

  voxblox::BlockIndexList blocks;
  layer.getAllAllocatedBlocks(&blocks);

  // Cache layer settings.
  size_t vps = layer.voxels_per_side();
  size_t num_voxels_per_block = vps * vps * vps;

  ohm::Voxel<float> ohm_voxel(&ohm_map, ohm_map.layout().occupancyLayer());
  if (!ohm_voxel.isLayerValid())
  {
    // Shouldn't really happen.
    return;
  }

  // Iterate over all blocks.
  for (const voxblox::BlockIndex &index : blocks)
  {
    // Iterate over all voxels in said blocks.
    const voxblox::Block<voxblox::OccupancyVoxel> &block = layer.getBlockByIndex(index);

    for (size_t linear_index = 0; linear_index < num_voxels_per_block; ++linear_index)
    {
      voxblox::Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
      const auto &voxel = block.getVoxelByLinearIndex(linear_index);

      const ohm::Key ohm_key = ohm_map.voxelKey(glm::dvec3(coord.x(), coord.y(), coord.z()));
      ohm::setVoxelKey(ohm_key, ohm_voxel);

      if (ohm_voxel.isValid())
      {
        auto voxel_data = ohm_voxel.data();
        voxel_data = (voxel.observed) ? voxel.probability_log : ohm::unobservedOccupancyValue();
        ohm_voxel.write(voxel_data);
      }
    }
  }

  ohm::save(filename.c_str(), ohm_map);
}
}  // namespace voxbloxpop

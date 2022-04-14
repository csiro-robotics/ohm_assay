// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
//
// Helper functions for incarnations of ohmpopros
#ifndef OHMPOPROS_ROSMAP_H
#define OHMPOPROS_ROSMAP_H

#include <ohm/Aabb.h>
#include <ohm/OccupancyMap.h>

#include <ohmdataros/RosDataSource.h>

#include <glm/vec3.hpp>

namespace ohmpopros
{
/// Ray data filter. Truncate rays to the (appropximate) bounding box region.
inline bool filterRays(glm::dvec3 *start, glm::dvec3 *end, unsigned *filter_flags,
                       const ohmdataros::RosDataSource &data_source, double ray_length_max)
{
  const glm::dvec3 sensor_position = data_source.sensorPosition();
  const glm::dvec3 half_extents(data_source.options().map_half_extents, data_source.options().map_half_extents,
                                (data_source.options().allow_vertical_extents) ?
                                  data_source.options().map_half_extents_vertical :
                                  data_source.options().map_half_extents);
  const ohm::Aabb local_map_aabb(sensor_position - half_extents, sensor_position + half_extents);
  if (ray_length_max > 0 && !ohm::clipRayFilter(start, end, filter_flags, ray_length_max))
  {
    return false;
  }
  if (!ohm::clipBounded(start, end, filter_flags, local_map_aabb))
  {
    return false;
  }
  return true;
}
}  // namespace ohmpopros

#endif  // OHMPOPROS_ROSMAP_H

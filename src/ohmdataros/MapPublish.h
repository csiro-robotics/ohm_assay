// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
//
// Helper functions for publishing a point cloud from various maps.
// Functions are inlined because of usage in various ohmpopmainxxxros.cpp files.
#ifndef OHMDATAROS_MAPPUBLISH_H
#define OHMDATAROS_MAPPUBLISH_H

#include <ohmdataros/PointCloud2Helper.h>

#include <ohmutil/Colour.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <functional>

namespace ohmdataros
{
/// Point to write out
struct Voxel
{
  double x;
  double y;
  double z;
  float r;
  float g;
  float b;
  float a;
  float scale;
  uint16_t intensity;
};

/// Colour generation function.
/// @param voxel Voxel to colourise.
using ColourSolver = std::function<ohm::Colour(const ohmdataros::Voxel &)>;

/// Generate a @c ColourSolver to apply uniform colour.
/// @param colour The colour to apply.
/// @return A colour solver function.
inline ColourSolver uniformColourSolver(const ohm::Colour &colour)
{
  return [colour](const ohmdataros::Voxel &) { return colour; };
}

/// Generate a @c ColourSolver to colour by intensity.
/// @return A colour solver function.
inline ColourSolver colourByIntensity()
{
  return [](const ohmdataros::Voxel &voxel) {
    const float shade = float(voxel.intensity) / float(0xffffu);
    ohm::Colour colour;
    colour.setRf(shade);
    colour.setGf(shade);
    colour.setBf(shade);
    colour.setAf(1.0f);
    return colour;
  };
}

/// Generate a @c ColourSolver to colour voxels by height. Since the height range is unknown, we use a @p height_cycle .
/// To avoid an abrupt change at the origin, we can @p offset_from_origin which will shift the zero height to the middle
/// of the colour range.
/// @param height_cycle The height colour range cycle.
/// @param offset_from_origin Offset the height cycle so that zero height is in the middle of the range.
/// @param colour_min Start of the colour cycle.
/// @param colour_max End of the colour cycle.
/// @param axis The height axis index: 0=>x, 1=>y, 2=>z (default)
/// @return A colour solver function.
inline ColourSolver colourByHeight(double height_cycle = 10.0, bool offset_from_origin = true,
                                   const ohm::Colour colour_min = ohm::Colour::kColours[ohm::Colour::kSpringGreen],
                                   const ohm::Colour colour_max = ohm::Colour::kColours[ohm::Colour::kRoyalBlue],
                                   int axis = 2)
{
  return [height_cycle, offset_from_origin, colour_min, colour_max, axis](const ohmdataros::Voxel &voxel) {
    // Set the colour by height, using a cycle determined by the argument.
    // note: fmod of a negate is equivalent to `-fmod(-value, mod)`
    double colour_height;
    switch (axis)
    {
    case 0:
      colour_height = voxel.x;
      break;
    case 1:
      colour_height = voxel.y;
      break;
    case 2:
    default:
      colour_height = voxel.z;
      break;
    }
    colour_height = colour_height - (offset_from_origin ? 0.5 * height_cycle : 0.0);
    float normalised_height = float(fmod(colour_height, height_cycle));
    // Fix negative values.
    normalised_height = (normalised_height >= 0) ? normalised_height : float(height_cycle + normalised_height);
    // Normalise range
    normalised_height /= float(height_cycle);
    // Select colour.
    const auto colour = ohm::Colour::lerp(colour_min, colour_max, normalised_height);
    return colour;
  };
}

/// Function signature for getting the next point in @c populateCloudMessage()
/// @param point The point structure to populate.
/// @return True if the call was valid for retreiving another point.
using CloudVoxelFunction = std::function<bool(Voxel &)>;

/// Populate a @c sensor_msgs::PointCloud2 message calling @p next_voxel until it returns false.
///
/// Supports X,Y,Z, intensity values from the @c Voxel structure, ignoring @c scale and colour.
///
/// - Does not modify @p cloud_msgs.header
/// - Does set the @p cloud_msgs.height (1), @p cloud_msgs.width to the point count and @p cloud_msg.point_step to the
///   point stride.
/// - Does clear any existing field data.
/// - Does clear any existing point data.
///
/// @param cloud_msg The point cloud message.
/// @param next_voxel Function to call to read voxel point.
/// @return The number of points added.
unsigned populateCloudMessage(sensor_msgs::PointCloud2 &cloud_msg, CloudVoxelFunction next_voxel);

/// Populate a @c visualization_msgs::Marker message calling @p next_voxel until it returns false.
///
/// Supports X,Y,Z and colour values @c Voxel structure. Uses the maximum @c Voxel::scale to set the overall cube scale.
///
/// Sets the following members of @p marker_msg . No other members are affected.
///
/// - Sets the @p marker_msg.pose to an identity pose
/// - Sets the @p marker_msg.type to @c visualization_msgs::Marker::CUBE_LIST
/// - Sets the @p marker_msg.action to @c visualization_msgs::Marker::MODIFY
/// - Sets the @p marker_msg.color to white.
/// - Sets the @p marker_msg.lifetime to zero.
/// - Sets the @p marker_msg.frame_locked to false.
/// - Sets the @p marker_msg.mesh_use_embedded_materialsto false.
/// - Clears and populates @p marker_msg.points
/// - Clears and populates @p marker_msg.colors
/// - Sets the @p marker_msg.scale to the maximum @c Voxel::scale (zero default)
///
/// @param marker_msg The point cloud message.
/// @param next_voxel Function to call to read each point.
/// @return The number of points added.
unsigned populateMarkerMessage(visualization_msgs::Marker &marker_msg, CloudVoxelFunction next_voxel);
}  // namespace ohmdataros

#endif  // OHMDATAROS_MAPPUBLISH_H

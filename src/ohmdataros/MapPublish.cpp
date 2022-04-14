// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "MapPublish.h"

namespace ohmdataros
{
unsigned populateCloudMessage(sensor_msgs::PointCloud2 &cloud_msg, CloudVoxelFunction next_voxel)
{
  // Publish position as float so RViz can show it.
  cloud_msg.data.clear();
  cloud_msg.fields.clear();
  const size_t x_idx = ohmdataros::addField<float>(cloud_msg.fields, "x");
  const size_t y_idx = ohmdataros::addField<float>(cloud_msg.fields, "y");
  const size_t z_idx = ohmdataros::addField<float>(cloud_msg.fields, "z");
  const size_t intensity_idx = ohmdataros::addField<uint16_t>(cloud_msg.fields, "intensity");

  Voxel vox{};
  unsigned point_count = 0;
  while (next_voxel(vox))
  {
    uint8_t *ptr = ohmdataros::appendPoint(cloud_msg.data, cloud_msg.fields);
    ohmdataros::writeField(ptr, cloud_msg.fields, x_idx, float(vox.x));
    ohmdataros::writeField(ptr, cloud_msg.fields, y_idx, float(vox.y));
    ohmdataros::writeField(ptr, cloud_msg.fields, z_idx, float(vox.z));
    ohmdataros::writeField(ptr, cloud_msg.fields, intensity_idx, vox.intensity);
    ++point_count;
  }

  const uint32_t point_size = ohmdataros::nextOffset(cloud_msg.fields.back());
  const size_t expected_data_size = size_t(point_count) * size_t(point_size);
  if (cloud_msg.data.size() == expected_data_size)
  {
    cloud_msg.width = point_count;
    cloud_msg.height = 1;
    cloud_msg.point_step = point_size;
    return point_count;
  }
  cloud_msg.data.clear();
  return 0;
}

unsigned populateMarkerMessage(visualization_msgs::Marker &marker_msg, CloudVoxelFunction next_voxel)
{
  marker_msg.points.clear();

  marker_msg.pose.position.x = marker_msg.pose.position.y = marker_msg.pose.position.z = 0;
  marker_msg.pose.orientation.x = marker_msg.pose.orientation.y = marker_msg.pose.orientation.z = 0;
  marker_msg.pose.orientation.w = 1;
  marker_msg.type = visualization_msgs::Marker::CUBE_LIST;
  marker_msg.action = visualization_msgs::Marker::MODIFY;
  marker_msg.color.r = marker_msg.color.g = marker_msg.color.b = marker_msg.color.a = 1.0f;
  marker_msg.lifetime = ros::Duration(0);
  marker_msg.frame_locked = false;
  marker_msg.mesh_use_embedded_materials = false;

  marker_msg.points.clear();
  marker_msg.colors.clear();

  Voxel vox{};
  unsigned point_count = 0;
  float scale = 0;
  while (next_voxel(vox))
  {
    marker_msg.points.emplace_back();
    marker_msg.colors.emplace_back();
    auto &pt = marker_msg.points.back();
    auto &colour = marker_msg.colors.back();
    pt.x = vox.x;
    pt.y = vox.y;
    pt.z = vox.z;
    colour.r = vox.r;
    colour.g = vox.g;
    colour.b = vox.b;
    colour.a = 1.0;
    scale = std::max(scale, vox.scale);
    ++point_count;
  }

  // Set the scale according to the last reported scale value.
  marker_msg.scale.x = marker_msg.scale.y = marker_msg.scale.z = scale;
  return point_count;
}
}  // namespace ohmdataros

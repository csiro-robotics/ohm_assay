// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OCTOMAPPOP_OCTOMAPPOP_H
#define OCTOMAPPOP_OCTOMAPPOP_H

#include <ohmapp/MapHarness.h>

#include <glm/vec2.hpp>

namespace octomap
{
class OcTree;
}

namespace octomappop
{
class OctomapPop : public ohmapp::MapHarness
{
public:
  /// Base class alias.
  using Super = ohmapp::MapHarness;

  struct MapOptions : public Super::MapOptions
  {
    float prob_hit = 0.9f;
    float prob_miss = 0.49f;
    float prob_thresh = 0.5f;
    double max_ray_length = -1.0;
    glm::vec2 prob_range = glm::vec2(0, 0);
    bool non_lazy_eval = false;
    bool collapse = false;
    bool offset_map_origin = false;

    MapOptions();

    void configure(cxxopts::OptionAdder &adder) override;
    void print(std::ostream &out) override;
  };

  /// Specialise collated options.
  struct Options : public Super::Options
  {
    inline MapOptions &map() { return static_cast<MapOptions &>(*map_); }
    inline const MapOptions &map() const { return static_cast<const MapOptions &>(*map_); }

    Options();
  };

  /// Constructor.
  OctomapPop(std::shared_ptr<ohmapp::DataSource> data_source);

  std::string description() const override;

  const Options &options() const { return static_cast<const Options &>(Super::options()); }

  Options &options() { return static_cast<Options &>(Super::options()); }

  octomap::OcTree *map() { return map_.get(); }
  const octomap::OcTree *map() const { return map_.get(); }

  const glm::dvec3 &mapOrigin() const { return map_origin_; }

protected:
  OctomapPop(std::unique_ptr<Options> &&options, std::shared_ptr<ohmapp::DataSource> data_source);

  int validateOptions(const cxxopts::ParseResult &parsed) override;
  int prepareForRun() override;
  bool processBatch(const glm::dvec3 &batch_origin, const std::vector<glm::dvec3> &sensor_and_samples,
                    const std::vector<double> &timestamps, const std::vector<float> &intensities,
                    const std::vector<glm::vec4> &colours) override;
  void finaliseMap() override;
  int saveMap(const std::string &path_without_extension) override;
  int saveCloud(const std::string &path_ply) override;
  void tearDown() override;

  std::unique_ptr<octomap::OcTree> map_;
  glm::dvec3 map_origin_ = glm::dvec3(0.0);
  /// Has anything been added to the map
  bool have_updated_ = false;
};
}  // namespace octomappop

#endif  // OCTOMAPPOP_OCTOMAPPOP_H

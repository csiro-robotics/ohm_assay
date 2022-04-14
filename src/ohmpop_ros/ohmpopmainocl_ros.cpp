// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "RosPopMain.h"

int main(int argc, char **argv)
{
  return ohmpoprosMainGpu(argc, argv, "ohmpopocl_ros", ohmpopros::paramToArgMappingsOhmOcl());
}

// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
//
// This is the program file for populating an Octomap occupancy map using SLAM data.
// The main entry point is near the bottom of the file with additional option parsing and support functions above that.
//
#include <ohmapp/SlamIOSource.h>
#include <ohmapp/ohmappmain.inl>

#include <octomap/octomap.h>
#include <octomappop/OctomapPop.h>

int main(int argc, char *argv[])
{
  return ohmappMain<octomappop::OctomapPop, ohmapp::SlamIOSource>(argc, argv);
}

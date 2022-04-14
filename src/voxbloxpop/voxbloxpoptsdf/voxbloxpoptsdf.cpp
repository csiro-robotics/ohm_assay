//
// author Kazys Stepanas
//
#include <ohmapp/SlamIOSource.h>
#include <ohmapp/ohmappmain.inl>

#include "voxbloxpop/VoxbloxPopMap.h"

int main(int argc, char *argv[])
{
  return ohmappMain<voxbloxpop::VoxbloxPopTsdf, ohmapp::SlamIOSource>(argc, argv);
}

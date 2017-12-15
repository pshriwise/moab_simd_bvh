#include<string>

#include "moab/ProgOptions.hpp"

#include "moab/Core.hpp"
#include "moab/CartVect.hpp"

#include "MBVHManager.h"

int main(int argc, char** argv) {

  moab::ErrorCode rval;

  moab::Interface *mbi = new moab::Core();

  // options handling
  ProgOptions po("A tool for firing rays on a DagMC geometry using a SIMD BVH.");

  //declare options
  bool build_stats, trav_stats;

  int vol_gid = 1, num_rand_rays = 1000;

  double rand_ray_center[3] = {0.0, 0.0, 0.0};
  double ray_center[3];
  double ray_dir[3];
  double rand_ray_radius;


  std::string filename;
  po.addRequiredArg<std::string>("MOAB Model", "Filename of the MOAB model.", &filename);
  
  po.addOpt<void>("build-stats,s", "Print the BVH structural statistics", &build_stats);
  po.addOpt<void>("trav-stats,S",  "Track and print BVH traversal statistics", &trav_stats);
  po.addOpt<int>("i",              "Specify the volume upon which to test ray intersections (default 1)", &vol_gid);
  po.addOpt<int>("n",              "Specify the number of random rays to fire (default 1000)", &num_rand_rays);

  po.addOpt<double>("x",   "Specify the x-value of random ray generation (default is zero)", rand_ray_center);
  po.addOpt<double>("y",   "Specify the y-value of random ray generation (default is zero)", rand_ray_center+1);
  po.addOpt<double>("z",   "Specify the z-value of random ray generation (default is zero)", rand_ray_center+2);

  po.addOpt<double>("r",   "Random ray radius. Random rays will begin at a distance r from the center of the random ray origin", &rand_ray_radius);

  po.addOpt<double>("cx",  "Specify the x-value of single ray generation", ray_center);
  po.addOpt<double>("cy",  "Specify the y-value of single ray generation", ray_center+1);
  po.addOpt<double>("cz",  "Specify the z-value of single ray generation", ray_center+2);
  po.addOpt<double>("cu",  "Specify the x-direction of single ray generation", ray_center);
  po.addOpt<double>("cv",  "Specify the y-direction of single ray generation", ray_center+1);
  po.addOpt<double>("cw",  "Specify the z-direction of single ray generation", ray_center+2);
  
  std::string python_dict;
  po.addOpt<std::string>("p", "if present, save parameters and results to a python dictionary file", &python_dict);

  po.addOptionHelpHeading("NOTE: any user-specified directions will be normalized before use");

  po.parseCommandLine(argc, argv);

  moab::Interface* MBI = new moab::Core();

  rval = MBI->load_file(filename.c_str());
  MB_CHK_SET_ERR(rval, "Failed to load file: " << filename << std::endl);

  MBVHManager* BVHManager = new MBVHManager(MBI);
  rval = BVHManager->build_all();
  MB_CHK_SET_ERR(rval, "Failed to build trees");
  
  
  
  return rval;
}

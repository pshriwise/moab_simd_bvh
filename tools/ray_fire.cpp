#include<string>
#include<math.h>
#include<ctime>

#include "moab/ProgOptions.hpp"

#include "moab/Core.hpp"
#include "moab/CartVect.hpp"

#include "MBVHManager.h"

#include "program_stats.hpp"

static const double PI = acos(-1.0);
static const double denom = 1.0 / ((double) RAND_MAX);
static const double denomPI = PI * denom;

inline void RNDVEC(moab::CartVect& uvw, const double& az = 0.0) {
  // denom normalizes rand values (see global defines)
  double theta = az * denom * rand(); // randomly samples from 0 to az. (Default az is 2PI)
  double u = 2 * denom * rand() - 1; // randomly samples from -1 to 1.
  uvw[0] = sqrt(1 - u * u) * cos(theta);
  uvw[1] = sqrt(1 - u * u) * sin(theta);
  uvw[2] = u;

}


moab::ErrorCode set_volume(moab::Interface* MBI, int vol_id, moab::EntityHandle &volume) {

  moab::ErrorCode rval;
  
  // obtain the specified volume
  moab::Tag gid_tag;
  rval = MBI->tag_get_handle(GLOBAL_ID_TAG_NAME, gid_tag);
  MB_CHK_SET_ERR(rval, "Failed to retrieve the global id tag");
  moab::Tag cat_tag;
  rval = MBI->tag_get_handle(CATEGORY_TAG_NAME, cat_tag);
  MB_CHK_SET_ERR(rval, "Failed to retrieve the category tag");
  char vol_name[CATEGORY_TAG_SIZE] = "Volume";
  const void* ptr[2] = { &vol_id, &(vol_name[0])};
  moab::Tag tags[2] = {gid_tag, cat_tag};
  moab::Range vols;
  rval = MBI->get_entities_by_type_and_tag(0, moab::MBENTITYSET, tags, ptr, 2, vols);
  MB_CHK_SET_ERR(rval, "Failed to retrieve the specified volume set");
  // there should be only one volume with this id
  if( 1 != vols.size() ) {
    MB_CHK_SET_ERR(moab::MB_FAILURE, "Incorrect number of volumes found for global id = "
		   << vol_id << std::endl << "Number found: " << vols.size() );
  }

  // set the volume
  volume = vols[0];

  return rval;
}  

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
  double rand_ray_radius = 0.0;


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

  po.addOpt<double>("cx",  "Specify the x-value of single ray generation");
  po.addOpt<double>("cy",  "Specify the y-value of single ray generation");
  po.addOpt<double>("cz",  "Specify the z-value of single ray generation");
  po.addOpt<double>("cu",  "Specify the x-direction of single ray generation");
  po.addOpt<double>("cv",  "Specify the y-direction of single ray generation");
  po.addOpt<double>("cw",  "Specify the z-direction of single ray generation");
  
  std::string python_dict;
  po.addOpt<std::string>("p", "if present, save parameters and results to a python dictionary file", &python_dict);

  po.addOptionHelpHeading("NOTE: any user-specified directions will be normalized before use");

  po.parseCommandLine(argc, argv);

  // create the MOAB instance and load the file
  moab::Interface* MBI = new moab::Core();
  rval = MBI->load_file(filename.c_str());
  MB_CHK_SET_ERR(rval, "Failed to load file: " << filename << std::endl);

  // initiate a BVH manager and build all trees for geometric entity sets
  MBVHManager* BVHManager = new MBVHManager(MBI);
  rval = BVHManager->build_all();
  MB_CHK_SET_ERR(rval, "Failed to build trees");

  moab::EntityHandle volume;
  rval = set_volume(MBI, vol_gid, volume);
  MB_CHK_SET_ERR(rval, "Failed to get and set the volume");

  // fire specified ray, if any
  if( po.getOpt("cx", ray_center)   &&
      po.getOpt("cy", ray_center+1) &&
      po.getOpt("cz", ray_center+2) &&
      po.getOpt("cu", ray_dir)      &&
      po.getOpt("cv", ray_dir+1)    &&
      po.getOpt("cw", ray_dir+2) ) {

    MBRay r(ray_center, ray_dir);
    r.instID = volume;

    rval = BVHManager->fireRay(volume, r);
    MB_CHK_SET_ERR(rval, "Failed to fire user-specified ray");
    
  }


  std::clock_t start;
  double duration = 0.0;
  
  /* Fire and time random rays */
  if(num_rand_rays > 0) {
    std::cout << "Firing " << num_rand_rays
	      << " random rays at volume " << vol_gid << "..." << std::flush;
  }
  
    MBRay *ray;
    moab::CartVect org, dir;
    int random_rays_missed = 0;
    org = moab::CartVect(rand_ray_center);
    for(int i = 0; i < num_rand_rays; i++){
      RNDVEC(dir);

      if( rand_ray_radius >= 0.0) {
	org = dir * rand_ray_radius + moab::CartVect(rand_ray_center);
      }
      
      ray = new MBRay(org.array(), dir.array());
      start = std::clock();
      BVHManager->fireRay(volume, *ray);
      duration += std::clock() - start;
      if(ray->geomID == -1) { random_rays_missed++; }
      delete ray;
    }

    /// REPORTING ///
    std::cout << std::endl;
    
    // repotr missed rays
    if(random_rays_missed) {
      std::cout << "Warning: " << random_rays_missed << " random rays did not hit the target volume" << std::endl;
    }

    if(build_stats) {
      BVHManager->tree_stats(volume);
    }
    
    // report on random rays if any were fired
    if (num_rand_rays > 0 ) {
      std::cout << "Total time per ray fire: " << duration / (double)CLOCKS_PER_SEC / num_rand_rays << " sec" << std::endl;

      report_memory_usage();
    }

    
  return rval;
}

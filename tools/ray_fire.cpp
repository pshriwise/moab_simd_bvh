#include<string>
#include<ctime>

#include "moab/ProgOptions.hpp"

#include "moab/Core.hpp"
#include "moab/CartVect.hpp"

#include "MBVHManager.h"

#include "program_stats.hpp"
#include "rayutil.hpp"

#include <fstream>

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

// function for reporting timing, memory, missed rays, etc.
void report(int rays_missed, int num_rays_fired, double duration);

int main(int argc, char** argv) {

  moab::ErrorCode rval;

  moab::Interface *mbi = new moab::Core();

  // options handling
  ProgOptions po("A tool for firing rays on a DagMC geometry using a SIMD BVH.");

  //declare options

  std::string filename;
  po.addRequiredArg<std::string>("MOAB Model", "Filename of the MOAB model.", &filename);

  bool build_stats = false;
  po.addOpt<void>("build-stats,s", "Print the BVH structural statistics", &build_stats);

  bool trav_stats = false;
  po.addOpt<void>("trav-stats,S",  "Track and print BVH traversal statistics", &trav_stats);

  int vol_gid = 1;
  po.addOpt<int>("i",              "Specify the volume upon which to test ray intersections (default 1)", &vol_gid);

  int num_rand_rays = 1000;
  po.addOpt<int>("n",              "Specify the number of random rays to fire (default 1000)", &num_rand_rays);

  double rand_ray_center[3] = {0.0, 0.0, 0.0};
  po.addOpt<double>("x",   "Specify the x-value of random ray generation (default is zero)", rand_ray_center);
  po.addOpt<double>("y",   "Specify the y-value of random ray generation (default is zero)", rand_ray_center+1);
  po.addOpt<double>("z",   "Specify the z-value of random ray generation (default is zero)", rand_ray_center+2);

  double rand_ray_radius = 0.0;
  po.addOpt<double>("r",   "Random ray radius. Random rays will begin at a distance r from the center of the random ray origin", &rand_ray_radius);

  std::string ray_file; // gets set later
  po.addOpt<std::string>("rf", "If provided, read rays from the provided csv file (x,y,z,u,v,w)");

  double ray_center[3]; // gets set later
  po.addOpt<double>("cx",  "Specify the x-value of single ray generation");
  po.addOpt<double>("cy",  "Specify the y-value of single ray generation");
  po.addOpt<double>("cz",  "Specify the z-value of single ray generation");

  double ray_dir[3];  // gets set later
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

  if(build_stats) {
    BVHManager->tree_stats(volume);
  }

  int rays_missed = 0;
  int rays_fired = 0;
  std::clock_t start;
  double duration = 0.0;

  // fire specified ray, if any
  if( po.getOpt("cx", ray_center)   && po.getOpt("cy", ray_center+1) && po.getOpt("cz", ray_center+2) &&
      po.getOpt("cu", ray_dir)      && po.getOpt("cv", ray_dir+1)    && po.getOpt("cw", ray_dir+2) ) {

    // construct ray
    MBRay ray(ray_center, ray_dir);
    ray.instID = volume;

    // fire the ray
    rays_fired++;
    start = std::clock();
    BVHManager->fireRay(ray);
    duration += std::clock() - start;
    MB_CHK_SET_ERR(rval, "Failed to fire user-specified ray");

    // check for intersection
    if(ray.geomID == -1) { rays_missed++; }
    std::cout << ray << std::endl;

    // report timing, memory, etc.
    report(rays_missed, rays_fired, duration);

  }
  // if the user provided a ray file,
  // then fire the rays from there and exit
  else if ( po.getOpt("rf", &ray_file) ) {

    // delimiter we're searching for in the CSV file
    std::string comma = ",";

    // open file
    std::ifstream file(ray_file);
    // object for storing line and ray values
    std::string line;
    double r[6];

    // loop over all lines in the file
    while( file.good() ){
      // extract the next line from the file
      getline(file, line);
      int index = 0;
      // extract all ray values from the line
      while( line.size() ) {
	// make sure our index hasn't gotten too high
	if( index >= 6 ) {
	  std::cout << "Too many values found in a row of the CSV file." << std::endl;
	  return 1;
	}
	// find the next comma
	size_t it = line.find(comma);
	// if no comma is found, assume we're at the end of the line,
	// treating the remainder of the line as a value
	if (it != std::string::npos) {
	  // extract the value from the line and store
	  std::string val = line.substr(0, it);
	  r[index] = stod(val);
	  // remove value and delimiter from the line
	  line.erase(0, it + comma.length());
	  // increment index
	  index++;
	}
	else {
	  // grab the last value in the line
	  r[index] = stod(line);
	  break;
	}
      }

      // construct a ray from the parsed values
      MBRay ray;
      ray.org = Vec3da(r[0], r[1], r[2]);
      ray.dir = Vec3da(r[3], r[4], r[5]);
      ray.instID = volume;

      rays_fired++;
      start = std::clock();
      rval = BVHManager->fireRay(ray);
      duration += std::clock() - start;
      MB_CHK_SET_ERR(rval, "Failed to fire ray");
      if(ray.geomID == -1) { rays_missed++; }
    }

  }
  // otherwise fire random rays from the ray center
  else if(num_rand_rays > 0) {
    std::cout << "Firing " << num_rand_rays
	      << " random rays at volume " << vol_gid << "..." << std::flush;


    MBRay ray;
    moab::CartVect org, dir;

    org = moab::CartVect(rand_ray_center);
    for(int i = 0; i < num_rand_rays; i++){
      RNDVEC(dir);

      if( rand_ray_radius > 0.0) {
	org = dir * rand_ray_radius + moab::CartVect(rand_ray_center);
      }

      // create the ray
      ray = MBRay(org.array(), dir.array());
      ray.instID = volume;

      // fire and time the ray
      rays_fired++;
      start = std::clock();
      rval = BVHManager->fireRay(ray);
      duration += std::clock() - start;
      MB_CHK_SET_ERR(rval, "Failed to fire ray");
      if(ray.geomID == -1) { rays_missed++; }
    }
  }

  report(rays_missed, rays_fired, duration);

  return rval;
}

void report(int rays_missed, int num_rays_fired, double duration) {
  /// REPORTING ///
  std::cout << std::endl;

  // repotr missed rays
  if(rays_missed) {
    std::cout << "Warning: " << rays_missed << " random rays did not hit the target volume" << std::endl;
  }

  // report on random rays if any were fired
  if(num_rays_fired > 0 ) {
    std::cout << "Total time per ray fire: " << duration / (double)CLOCKS_PER_SEC / num_rays_fired << " sec" << std::endl;
  }

  // display estimate of program memory usage
  report_memory_usage();
}

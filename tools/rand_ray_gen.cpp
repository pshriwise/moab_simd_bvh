


#include <iostream>
#include <string>
#include <sstream>
#include "moab/ProgOptions.hpp"
#include "rayutil.hpp"

int main (int argc, char** argv) {

  // setup program options
  ProgOptions po("A tool for generating csv output of random rays. Use redirect (>) to capture in file.");

  double cx = 0.0, cy = 0.0, cz = 0.0;
  po.addOpt<double>("cx",  "Specify the x-value of ray generation origin (default: 0.0)", &cx);
  po.addOpt<double>("cy",  "Specify the y-value of ray generation origin (default: 0.0)", &cy);
  po.addOpt<double>("cz",  "Specify the z-value of ray generation origin (default: 0.0)", &cz);

  int num_rays = 100;
  po.addOpt<int>("n", "Number of random rays to generate (default: 1000)", &num_rays);

  
  po.parseCommandLine(argc, argv);

  // loop and write rays
  for( int i = 0; i < num_rays; i++ ) {

    // generate a random direction
    moab::CartVect rand_dir;
    RNDVEC(rand_dir);

    // create the ray string (x,y,z,u,v,w)
    std::stringstream ray;
    ray << cx << "," << cy << "," << cz << ",";
    ray << rand_dir[0] << "," << rand_dir[1] << "," << rand_dir[2];

    // write the ray to screen
    std::cout << ray.str() << std::endl;
  }
  
  return 0;
}
  


#include "test_files.h"

#include "assert.h"

#include "moab/Core.hpp"
#include "moab/CartVect.hpp"

#include "Builder.h"
#include "Intersector.h"
#include "MBVH.h"

#include "testutil.hpp"

static const float PI = acos(-1.0);
static const float denom = 1.0 / ((float) RAND_MAX);
static const float denomPI = PI * denom;

inline void RNDVEC(Vec3fa& uvw, float &az)
{
  // denom normalizes rand values (see global defines)
  double theta = az * denom * rand(); // randomly samples from 0 to az. (Default az is 2PI)
  double u = 2 * denom * rand() - 1; // randomly samples from -1 to 1.
  uvw[0] = sqrt(1-u*u)*cos(theta);
  uvw[1] = sqrt(1-u*u)*sin(theta);
  uvw[2] = u;
}

moab::ErrorCode test_file(std::string filename) {
  moab::Interface* mbi = new moab::Core();
  
  moab::ErrorCode rval;
  
  rval = mbi->load_file(filename.c_str());
  MB_CHK_SET_ERR(rval, "Failed to load test file");

  moab::Range all_tris;

  rval = mbi->get_entities_by_type(0, moab::MBTRI, all_tris, true);
  MB_CHK_SET_ERR(rval, "Failed to get all triangles in the model");

  std::vector<TriangleRef> tris;

  moab::Range::iterator i;
  int counter = 0;
  for(i = all_tris.begin(); i != all_tris.end(); i++) {

    moab::EntityHandle tri = *i;

    TriangleRef *t = new TriangleRef(tri, mbi);
    
    tris.push_back(*t);
    
  }

  TriangleBVH TBVH(create_leaf);

  BuildSettings settings;
  
  BuildStateTri bs = BuildStateTri(0, tris);

  NodeRef* root = TBVH.Build(settings, bs);
  
  TriIntersector TINT;

  Vec3fa org(0.0, 0.0, 0.0);

  float angle = 2*PI;
  
  int num_rand_rays = 1e4;
  
  for(unsigned int i = 0; i < num_rand_rays; i++) {
    
    Vec3fa dir;
    //    dir = Vec3fa(-0.660058, 0.750736, 0.0268018);
    RNDVEC(dir, angle);
    dir.normalize();
    
    Ray r = Ray(org, dir, 0.0, inf);
    
    TINT.intersectRay(*root, r);

    if(r.primID == -1) {
      std::cout << "Ray number: " << i << std::endl;
      std::cout << r << std::endl;
    }
    
    CHECK(r.tfar < (float)inf);
    
  }

 delete mbi;
 delete root;

}

moab::ErrorCode test_cube() {
  
  moab::Interface* mbi = new moab::Core();
  
  moab::ErrorCode rval;
  
  rval = mbi->load_file(TEST_CUBE);
  MB_CHK_SET_ERR(rval, "Failed to load test file");

  moab::Range all_tris;

  rval = mbi->get_entities_by_type(0, moab::MBTRI, all_tris, true);
  MB_CHK_SET_ERR(rval, "Failed to get all triangles in the model");

  std::vector<TriangleRef> tris;

  moab::Range::iterator i;
  int counter = 0;
  for(i = all_tris.begin(); i != all_tris.end(); i++) {

    moab::EntityHandle tri = *i;

    TriangleRef t = TriangleRef(tri, mbi);
    
    tris.push_back(t);
    
  }


  TriangleBVH TBVH(create_leaf);

  BuildSettings settings;
  
  BuildStateTri bs = BuildStateTri(0, tris);

  NodeRef* root = TBVH.Build(settings, bs);

  TriIntersector TINT;

  Vec3fa org(0.0,0.0,0.0);
  Vec3fa dir(1.0,0.0,0.0);

  Ray ray(org, dir, 0.0, inf);
  
  TINT.intersectRay(*root, ray);
  
  CHECK_REAL_EQUAL(ray.tfar, 5.0f, 1e-6);

  dir = Vec3fa(0.0,1.0,0.0);

  ray = Ray(org, dir, 0.0, inf);
  
  TINT.intersectRay(*root, ray);
  
  CHECK_REAL_EQUAL(ray.tfar, 5.0f, 1e-6);

  dir = Vec3fa(0.0,0.0,1.0);

  ray = Ray(org, dir, 0.0, inf);
  
  TINT.intersectRay(*root, ray);
  
  CHECK_REAL_EQUAL(ray.tfar, 5.0f, 1e-6);

  delete mbi;
  delete root;
}

int main(int argc, char** argv) {

  int num_fails = 0;
  std::cout << "General test for cube model...";
  num_fails += test_file(TEST_CUBE);
  std::cout << "done" << std::endl;
  std::cout << "General test for 3K triangle cube model...";
  num_fails += test_file(TEST_3K_CUBE);
  std::cout << "done" << std::endl;
  std::cout << "Specific test for cube model...";
  num_fails += test_cube();
  std::cout << "done" << std::endl;
  std::cout << "General test for sphere model...";
  num_fails += test_file(TEST_SPHERE);
  std::cout << "done" << std::endl;
  
return num_fails;

}

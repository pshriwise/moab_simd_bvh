

#include "assert.h"

#include "moab/Core.hpp"
#include "moab/CartVect.hpp"

#include "Primitive.h"
#include "Builder.h"
#include "Intersector.h"

int main(int argc, char** argv) {
  
  moab::Interface* mbi = new moab::Core();
  
  moab::ErrorCode rval;
  
  rval = mbi->load_file("sphere.h5m");
  MB_CHK_SET_ERR(rval, "Failed to load test file");

  moab::Range all_tris;

  rval = mbi->get_entities_by_type(0, moab::MBTRI, all_tris, true);
  MB_CHK_SET_ERR(rval, "Failed to get all triangles in the model");

  std::vector<BuildPrimitive> primitives;

  moab::Range::iterator i;
  int counter = 0;
  for(i = all_tris.begin(); i != all_tris.end(); i++) {

    moab::EntityHandle tri = *i;

    std::vector<moab::EntityHandle> conn;
    rval = mbi->get_connectivity(&tri, 1, conn);
    MB_CHK_SET_ERR(rval, "Failed to get the connectivity of a triangle");

    assert(conn.size() == 3);
    moab::CartVect coords[3];
    
    rval = mbi->get_coords(&(conn[0]), 3, coords[0].array() );
    MB_CHK_SET_ERR(rval, "Failed to get vertex coordinates of triangle");

    conn.clear();


    Vec3fa max(neg_inf), min(inf);
    
    max.x = std::max(coords[0][0],std::max(coords[1][0], coords[2][0]));
    max.y = std::max(coords[0][1],std::max(coords[1][1], coords[2][1]));
    max.z = std::max(coords[0][2],std::max(coords[1][2], coords[2][2]));
		     
    min.x = std::min(coords[0][0],std::min(coords[1][0], coords[2][0]));
    min.y = std::min(coords[0][1],std::min(coords[1][1], coords[2][1]));
    min.z = std::min(coords[0][2],std::min(coords[1][2], coords[2][2]));

    BuildPrimitive p(min.x, min.y, min.z, 0,
		     max.x, max.y, max.z, counter++);
    primitives.push_back(p);
    
  }


  BuildPrimitiveBVH BPBVH(create_leaf);

  BuildSettings settings;
  
  BuildState bs = BuildState(0, primitives);

  NodeRef* root = BPBVH.Build(settings, bs);

  Vec3fa org(0.0, 0.0, 0.0);
  Vec3fa dir(1.0, 0.0, 0.0);

  Ray r = Ray(org, dir, 0.0, inf);

  BuildPrimitiveIntersector BPINT;

  BPINT.intersectRay(*root, r);
  
  assert(r.tfar < inf);

  std::cout << r << std::endl;
  
return 0;

}

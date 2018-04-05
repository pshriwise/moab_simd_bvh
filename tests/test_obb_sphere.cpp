
#include "testutil.hpp"
#include "OBB.h"
#include "test_files.h"

#include "moab/Core.hpp"


int main() {

  moab::Interface * MBI = new moab::Core();

  moab::ErrorCode rval = MBI->load_file(TEST_SMALL_SPHERE);
  MB_CHK_SET_ERR(rval, "Failed to load the test file.");

  moab::Range all_verts;
  rval = MBI->get_entities_by_type(0, moab::MBVERTEX, all_verts);
  MB_CHK_SET_ERR(rval, "Failed to get all vertices in the model.");

  std::vector<double> x, y, z;
  x.resize(all_verts.size()); y.resize(all_verts.size()); z.resize(all_verts.size());

  rval = MBI->get_coords(all_verts, &(x[0]), &(y[0]), &(z[0]));
  MB_CHK_SET_ERR(rval, "Failed to get vertex coordinates.");

  std::vector<float> xf, yf, zf;
  
  for(size_t i = 0; i < all_verts.size(); i++) {
    for(size_t j = 0; j < 7; j++) {
    xf.push_back(x[i]);
    yf.push_back(y[i]);
    zf.push_back(z[i]);
    }
  }
			 
  OBB box(&xf.front(), &yf.front(), &zf.front(), all_verts.size());
  
  for(size_t i = 0; i < all_verts.size(); i++) {
    Vec3fa pnt(xf[i], yf[i], zf[i]);
    CHECK(inside(box, pnt));
  }
  
  return 0;
}

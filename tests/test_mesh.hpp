

#include "Primitive.h"



void build_hollow_cube(const float& x_min, const float& x_width, const size_t& x_prims,
		       const float& y_min, const float& y_width, const size_t& y_prims,
		       const float& z_min, const float& z_width, const size_t& z_prims,
		       BuildPrimitive* primitives, size_t& numPrimitives) {

  std::vector<BuildPrimitive> prims;
  
  float x_step = x_width/(float)x_prims;
  float y_step = y_width/(float)y_prims;
  float z_step = z_width/(float)z_prims;

  int prim_id = 0;
  
  for(size_t i = 0; i <= x_prims; i++) {
    for(size_t j = 0; j <= y_prims; j++) {
      for(size_t k = 0; k <= z_prims;) {

	if ( !((i == 0 || i == x_prims) &&
	       (j == 0 || j == y_prims)) ) {	  
	  if (k == 1) {
	    k = z_prims; continue;
	  }
	}

        BuildPrimitive p = BuildPrimitive(x_min+i*x_step,
					  y_min+j*y_step,
					  z_min+k*y_step,
					  0,
					  x_min+(i+1)*x_step,
					  y_min+(j+1)*y_step,
					  z_min+(k+1)*z_step,
					  prim_id++);

	prims.push_back(p);

	k++;
	
      }
    }
  }

  primitives = &prims.front();
  numPrimitives = prims.size();
}

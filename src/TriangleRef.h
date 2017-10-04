#pragma once

#include "Primitive.h"

#include "moab/Core.hpp"
#include "moab/CartVect.hpp"

struct TriangleRef : public BuildPrimitive {
  using::BuildPrimitive::lower;
  using::BuildPrimitive::upper;
  
  inline TriangleRef(long unsigned int tri_handle): eh(tri_handle){}

  inline TriangleRef(long unsigned int tri_handle, moab::Interface* mbi): eh(tri_handle) { set_bounds(mbi); }

  inline void set_bounds(moab::Interface * mbi) {
    
    std::vector<moab::EntityHandle> conn;
    moab::ErrorCode rval = mbi->get_connectivity(&eh, 1, conn);
    MB_CHK_SET_ERR_RET(rval, "Failed to get triangle connectivity.");

    assert(conn.size() == 3);

    moab::CartVect coords[3];
    
    rval = mbi->get_coords(&(conn[0]), 3, coords[0].array() );
    MB_CHK_SET_ERR_RET(rval, "Failed to get triangle vert coords");

    
    upper.x = std::max(coords[0][0],std::max(coords[1][0], coords[2][0]));
    upper.y = std::max(coords[0][1],std::max(coords[1][1], coords[2][1]));
    upper.z = std::max(coords[0][2],std::max(coords[1][2], coords[2][2]));
		     
    lower.x = std::min(coords[0][0],std::min(coords[1][0], coords[2][0]));
    lower.y = std::min(coords[0][1],std::min(coords[1][1], coords[2][1]));
    lower.z = std::min(coords[0][2],std::min(coords[1][2], coords[2][2]));
    
  }
  
  long unsigned int eh;

};
  
  

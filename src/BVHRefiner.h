
#pragma once


#include "Node.h"
#include "moab/Core.hpp"

template <typename V, typename T, typename I>
class BVHRefinerT {

  typedef BuildStateT<PrimRef> BuildState;

  typedef MBTriangleRefT<V, T, I> P;
  
 public:
  inline BVHRefinerT(MOABDirectAccessManager* MDAM) : MDAM(MDAM) { OBVHBuilder = new OBVH<V,T,I>(MDAM);  }
  
 private:
  
  inline bool is_hv_region(const BuildState& cur) {
    bool result = false;
    // create a set of the primitive EntityHandles
    moab::Range ehs;
    for(size_t i = 0; i < cur.size(); i++) {
      	P t = P((I*)MDAM->conn + (cur.prims[i].primID()*MDAM->element_stride), (I)cur.prims[i].primitivePtr);
	ehs.insert(t.eh);
    }

    moab::Range triangle_verts;
    moab::ErrorCode rval = MDAM->MOAB_instance->get_connectivity(ehs, triangle_verts);
    MB_CHK_SET_ERR_CONT(rval, "Failed to get connectivity when checking for HV region.");
    
    for(moab::Range::iterator it = triangle_verts.begin(); it != triangle_verts.end(); it++) {
      moab::EntityHandle vert = *it;

      moab::Range adj_tris;
      rval = MDAM->MOAB_instance->get_adjacencies(&vert, 1, 2, false, adj_tris);
      MB_CHK_SET_ERR_CONT(rval, "Failed to get adjacencies when checking for HV region.");

      moab::Range overlap;
      overlap -= adj_tris;
      if( (double)overlap.size()/(double)ehs.size() < 0.8 ) {
	result = true;
	break;
      }
    }

    return result;
  }

 public:
  
  inline NodeRef* refine(BuildState &cur) {
    bool is_hv = is_hv_region(cur);
    if( is_hv ) {
      return OBVHBuilder->Build(cur);
    }
    else {
      return NULL;
    }
  }

 private:
  
  MOABDirectAccessManager* MDAM;
  OBVH<V,T,I>* OBVHBuilder;
  
};

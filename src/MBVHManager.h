#pragma once

#include <iostream>
#include <string>

#include "moab/Core.hpp"
#include "MBTagConventions.hpp"

#include "MBVH.h"
#include "MOABDirectAccessManager.h"

inline void output_w_border(std::string message) {
  std::string border(message.size(), '=');
  std::cout << std::endl << border << std::endl << message << std::endl << border << std::endl;
  return;
}
  

class MBVHManager {
 public:
  
  moab::Interface* MBI;
  moab::ErrorCode rval;

  MOABDirectAccessManager* MDAM;
  MBVH* MOABBVH;
  
  std::vector<NodeRef*> BVHRoots;
  moab::EntityHandle lowest_set;

  moab::Tag geom_dim_tag;
  
  MBVHManager(moab::Interface* moab) : MBI(moab), rval(moab::MB_SUCCESS), MDAM(NULL)
  {
    initialize();
  };


  void initialize() {
    // get all entities of dim 2 (facet entities)
    moab::Range facets;
    rval = MBI->get_entities_by_dimension(0, 2, facets, true);
    MB_CHK_SET_ERR_CONT(rval, "Failed to get all facet entities");

    // get all entities of dim 0 (vertices)
    moab::Range verts;
    rval = MBI->get_entities_by_dimension(0, 0, verts, true);
    MB_CHK_SET_ERR_CONT(rval, "Failed to get all vertex entities");

    // check that facet and vert space is contiguous
    if(facets.psize() != 1 && verts.psize() != 1) {
      output_w_border("WARNING: EntityHandle Space is not contiguous. Use of this tool is not valid. Please see the ReorderTool to correct this issue");
    }

    // check that all entities of dimension 2 are triangles
    if(!facets.all_of_type(moab::MBTRI)) {
      output_w_border("WARNING: EntityHandle Space is not contiguous. Use of this tool is not valid. Please see the ReorderTool to correct this issue");
    }      

    // get the facet connectivity direct access pointer
    int vpere, element_count;
    moab::EntityHandle *connPointer;
    rval = MBI->connect_iterate(facets.begin(), facets.end(), connPointer, vpere, element_count);
    MB_CHK_SET_ERR_CONT(rval, "Failed to get the direct access pointer for facets");

    // get the vertex coordinate direct access pointer    
    double *x_ptr, *y_ptr, *z_ptr;
    int vert_count;
    rval = MBI->coords_iterate(verts.begin(), verts.end(), x_ptr, y_ptr, z_ptr, vert_count);
    MB_CHK_SET_ERR_CONT(rval, "Failed to retrieve the vertex pointer");

    MDAM = new MOABDirectAccessManager(MBI, x_ptr, y_ptr, z_ptr, vert_count, facets.front(), connPointer, element_count, vpere);


    rval = MBI->tag_get_handle(GEOM_DIMENSION_TAG_NAME, geom_dim_tag);
    MB_CHK_SET_ERR_CONT(rval, "Failed to get the geom dim tag handle");

    moab::Range surfs, vols;
    int dim = 2;
    void *ptr = &dim;
    rval = MBI->get_entities_by_type_and_tag(0, moab::MBENTITYSET, &geom_dim_tag, &ptr, 1, surfs);
    MB_CHK_SET_ERR_CONT(rval, "Failed to retrieve surface entitysets");

    dim = 3;
    rval = MBI->get_entities_by_type_and_tag(0, moab::MBENTITYSET, &geom_dim_tag, &ptr, 1, vols);
    MB_CHK_SET_ERR_CONT(rval, "Failed to retrieve surface entitysets");

    moab::Range all_sets = unite(surfs,vols);

    BVHRoots = std::vector<NodeRef*>((all_sets.back() - all_sets.front())+1);
    lowest_set = all_sets.front();
    
    MOABBVH = new MBVH(MDAM);
  };

  void tree_stats(moab::EntityHandle ent);
  
  NodeRef* get_root(moab::EntityHandle ent);
  
  moab::ErrorCode build( moab::Range geom_sets);
  
  moab::ErrorCode build_all();

  moab::ErrorCode fireRay(MBRay &ray);

  moab::ErrorCode fireRaySurf(MBRay &ray);

  moab::ErrorCode closestToLocation(MBRay & ray);

  moab::ErrorCode closestToLocationSurf(MBRay & ray);

};
  

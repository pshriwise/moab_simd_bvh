

#include <iostream>
#include <string>

#include "moab/Core.hpp"
#include "MBTagConventions.hpp"

#include "MBVH.h"
#include "MOABDirectAccessManager.h"

void output_w_border(std::string message) {
  std::string border(message.size(), '=');
  std::cout << std::endl << border << std::endl << message << std::endl << border << std::endl;
  return;
}
  

struct MBVHManager {

  moab::Interface* MBI;
  moab::ErrorCode rval;

  MOABDirectAccessManager* MDAM;

  std::vector<NodeRef*> BVHRoots;
  moab::EntityHandle lowest_set;

  moab::Tag geom_dim_tag;
  
  MBVHManager(moab::Interface* moab) : MBI(moab), rval(moab::MB_SUCCESS), MDAM(NULL)
  {
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
    int vpere, numPrimitives;
    moab::EntityHandle *connPointer;
    rval = MBI->connect_iterate(facets.begin(), facets.end(), connPointer, vpere, numPrimitives);
    MB_CHK_SET_ERR_CONT(rval, "Failed to get the direct access pointer for facets");

    // get the vertex coordinate direct access pointer    
    double *x_ptr, *y_ptr, *z_ptr;
    int vert_count;
    rval = MBI->coords_iterate(verts.begin(), verts.end(), x_ptr, y_ptr, z_ptr, vert_count);
    MB_CHK_SET_ERR_CONT(rval, "Failed to retrieve the vertex pointer");

    MDAM = new MOABDirectAccessManager(0, x_ptr, y_ptr, z_ptr, connPointer);

    moab::Range surfs, vols;

    rval = MBI->tag_get_handle(GEOM_DIMENSION_TAG_NAME, geom_dim_tag);
    MB_CHK_SET_ERR_CONT(rval, "Failed to get the geom dim tag handle");

    int dim = 2;
    void *ptr = &dim;
    rval = MBI->get_entities_by_type_and_tag(0, moab::MBENTITYSET, &geom_dim_tag, &ptr, 1, surfs);
    MB_CHK_SET_ERR_CONT(rval, "Failed to retrieve surface entitysets");

    dim = 3;
    rval = MBI->get_entities_by_type_and_tag(0, moab::MBENTITYSET, &geom_dim_tag, &ptr, 1, vols);
    MB_CHK_SET_ERR_CONT(rval, "Failed to retrieve surface entitysets");

    moab::Range all_sets = unite(surfs,vols);
    // if all sets are contiguous, then setup offset lookup
    if(all_sets.psize() == 1) {
      BVHRoots = std::vector<NodeRef*>(surfs.size() + vols.size());
      lowest_set = all_sets.front();
    }
    // otherwise, panic (for now)
    else {
      output_w_border("Geometric EntitySets are not contiguous");
    }
  };

};

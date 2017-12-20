

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
  MBVH* MOABBVH;
  
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
    // if all sets are contiguous, then setup offset lookup
    if(all_sets.psize() == 1) {
      BVHRoots = std::vector<NodeRef*>(surfs.size() + vols.size());
      lowest_set = all_sets.front();
    }
    // otherwise, panic (for now)
    else {
      output_w_border("Geometric EntitySets are not contiguous");
    }

    MOABBVH = new MBVH(MDAM);
   
  };
  
  NodeRef* get_root(moab::EntityHandle ent) {
    assert(ent - lowest_set > 0 && ent - lowest_set < BVHRoots.size());
    NodeRef* root = BVHRoots[ent - lowest_set];
    return root;
  }

  void tree_stats(moab::EntityHandle ent) {
    BVHStatTracker *BVHS = new BVHStatTracker();
    NodeRef* root = get_root(ent);
    BVHS->gatherStats(*root);
  }

  
  moab::ErrorCode build( moab::Range geom_sets) {

    // make sure that we're working only with EntitySets here
    assert(geom_sets.all_of_type(moab::MBENTITYSET));

    moab::Range::iterator ri;
    std::vector<moab::Tag> temp_tags;
    
    for(ri = geom_sets.begin(); ri != geom_sets.end(); ri++) {
      
      // make sure this is a geometric entityset by checking for tag
      rval = MBI->tag_get_tags_on_entity(*ri, temp_tags);
      MB_CHK_SET_ERR(rval, "Failed to get all tags on EntitySet: " << *ri);
      assert(std::find(temp_tags.begin(), temp_tags.end(), geom_dim_tag) != temp_tags.end());

      //check for existing tree
      if(BVHRoots[*ri - lowest_set]) {
	continue;
      }
      
      int dim = 0;
      rval = MBI->tag_get_data(geom_dim_tag, &(*ri), 1, &dim);
      MB_CHK_SET_ERR(rval, "Failed to get the geom dimension of EntitySet: " << *ri);

      moab::Range tris;
      moab::Range child_surfs;
      std::vector<NodeRef*> sets;
      NodeRef* root;

      moab::Tag sense_tag;
      moab::EntityHandle data[2];
      
      switch (dim) {

	case 2 : // build surface tree
	  // create a new tree with the MBVH class;
	  rval = MBI->get_entities_by_type(*ri, moab::MBTRI, tris);
	  MB_CHK_SET_ERR(rval, "Failed to get triangles for surface: " << *ri);

	  // IN PROGRESS
	  root = MOABBVH->Build(tris.front(), tris.front() - MDAM->first_element, tris.size());
	  if(!root) { MB_CHK_SET_ERR(moab::MB_FAILURE, "Failed to build BVH for surface: " << *ri); }

	  BVHRoots[*ri - lowest_set] = root;

	  //update the root node to a setLeaf node

	  //first get the sense information

	  rval = MBI->tag_get_handle("GEOM_SENSE_2", sense_tag);
	  MB_CHK_SET_ERR(rval, "Failed to get the sense tag");

	  rval = MBI->tag_get_data(sense_tag, &(*ri), 1, (void*)data);
	  MB_CHK_SET_ERR(rval, "Failed to get the sense data");

	  MOABBVH->makeSetNode(root, (*ri), data[0], data[1]);

	  break;
	  
	case 3 : //create volume tree from child surface trees
	  // get the volume's child surfaces
	  rval = MBI->get_child_meshsets(*ri, child_surfs);
	  MB_CHK_SET_ERR(rval, "Failed to get child surfaces of volume" << *ri);

	  for(unsigned int i = 0; i < child_surfs.size(); i++){
	    // make sure there are trees for all of these surfaces
	    rval = build( child_surfs );
	    MB_CHK_SET_ERR(rval, "Failed to build child surface trees of volume " << *ri);
	    
	    sets.push_back(BVHRoots[child_surfs[i] - lowest_set]);
	  }

	  // join the trees here
	  root = MOABBVH->join_trees( sets );
	  if(!root) { MB_CHK_SET_ERR(moab::MB_FAILURE, "Failed to build BVH for volume: " << *ri); }
	  BVHRoots[*ri - lowest_set] = root;	  	  
	  break;
	  
	default:
	  MB_CHK_SET_ERR(moab::MB_FAILURE, "Entity " << *ri << "is not a surface or volume EntitySet");
	}
    }
    
    return rval;
  }

  inline moab::ErrorCode build_all() {
    moab::ErrorCode rval;

    moab::Tag geom_dim_tag;
    rval = MBI ->tag_get_handle(GEOM_DIMENSION_TAG_NAME, geom_dim_tag);
    MB_CHK_SET_ERR_CONT(rval, "Failed to get the geom dim tag handle");
    
    moab::Range all_vols;
    
    int dim = 3;
    void *ptr = &dim;
    rval = MBI ->get_entities_by_type_and_tag(0, moab::MBENTITYSET, &geom_dim_tag, &ptr, 1, all_vols);
    MB_CHK_SET_ERR_CONT(rval, "Failed to retrieve surface entitysets");

    rval = build(all_vols);
    MB_CHK_SET_ERR(rval, "Failed to build trees for all volumes");

    return rval;
  }


  inline moab::ErrorCode fireRay(const moab::EntityHandle &set, MBRay &ray) {
    NodeRef* root = BVHRoots[set - lowest_set];
    if(!root) { MB_CHK_SET_ERR(moab::MB_FAILURE, "Failed to retrieve the root for EntitySet " << set); }
    MOABBVH->intersectRay(*root, ray);
    return moab::MB_SUCCESS;
  }
    
};
  

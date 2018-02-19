

#include "MBVHManager.h"
#include "moab/Core.hpp"

NodeRef* MBVHManager::get_root(moab::EntityHandle ent) {
    assert(ent - lowest_set >= 0 && ent - lowest_set < BVHRoots.size());
    NodeRef* root = BVHRoots[ent - lowest_set];
    return root;
  }

void MBVHManager::tree_stats(moab::EntityHandle ent) {
    BVHStatTracker *BVHS = new BVHStatTracker();
    NodeRef* root = get_root(ent);
    BVHS->gatherStats(*root);
  }

moab::ErrorCode MBVHManager::build( moab::Range geom_sets) {

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

      std::vector<moab::EntityHandle> tris;
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
	  root = MOABBVH->Build(&(tris[0]), tris.size());
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

moab::ErrorCode MBVHManager::build_all() {
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


moab::ErrorCode MBVHManager::fireRay( MBRay &ray ) {
<<<<<<< e2dfc66004f169b791f7983d3435b1214bf184a6
  NodeRef* root = get_root(ray.instID);
  if(!root) { MB_CHK_SET_ERR(moab::MB_FAILURE, "Failed to retrieve the root for EntitySet " << ray.instID); }
  MOABBVH->intersectRay(*root, ray);
  return moab::MB_SUCCESS;
}

moab::ErrorCode MBVHManager::fireRaySurf( MBRay &ray ) {
  NodeRef* root = get_root(ray.geomID);
  if(!root) { MB_CHK_SET_ERR(moab::MB_FAILURE, "Failed to retrieve the root for EntitySet " << ray.geomID); }
  MOABBVH->intersectRay(*root, ray);
  return moab::MB_SUCCESS;
}

moab::ErrorCode MBVHManager::closestToLocation( MBRay &ray ) {
  NodeRef *root = get_root(ray.instID);
  if(!root) { MB_CHK_SET_ERR(moab::MB_FAILURE, "Failed to retrieve the root for EntitySet " << ray.instID); }
  MOABBVH->intersectClosest(*root, ray);
  return moab::MB_SUCCESS;
}
  
moab::ErrorCode MBVHManager::closestToLocationSurf( MBRay &ray ) {
  NodeRef* root = get_root(ray.geomID);
  if(!root) { MB_CHK_SET_ERR(moab::MB_FAILURE, "Failed to retrieve the root for EntitySet " << ray.geomID); }
  MOABBVH->intersectClosest(*root, ray);
  return moab::MB_SUCCESS;
}

=======
      NodeRef* root = get_root(ray.instID);
    if(!root) { MB_CHK_SET_ERR(moab::MB_FAILURE, "Failed to retrieve the root for EntitySet " << ray.instID); }
    MOABBVH->intersectRay(*root, ray);
    return moab::MB_SUCCESS;
  }

moab::ErrorCode MBVHManager::fireRaySurf( MBRay &ray ) {
    NodeRef* root = get_root(ray.geomID);
    if(!root) { MB_CHK_SET_ERR(moab::MB_FAILURE, "Failed to retrieve the root for EntitySet " << ray.geomID); }
    MOABBVH->intersectRay(*root, ray);
    return moab::MB_SUCCESS;
  }
>>>>>>> Adding shared library to build.

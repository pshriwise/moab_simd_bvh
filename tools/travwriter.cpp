

#include "MBVH.h"
#include "MBVHManager.h"

#include "moab/Core.hpp"
#include "moab/ProgOptions.hpp"

#include "WriteVisitor.hpp"
#include "TraversalClass.hpp"

#include <string>
#include <sstream>
#include <iomanip>

typedef BVHCustomTraversalT<Vec3da, double, moab::EntityHandle> BVHCustomTraversal;

class TravWriter : public WriteVisitor {

public:

  TravWriter(moab::Interface* original_moab_instance) : WriteVisitor(original_moab_instance),
							nodes_visited(0) {
  }
  
private:
  // some counters
  int nodes_visited;
  
public:
  
  virtual bool visit(NodeRef current_node, TravRay& vray, const vfloat4& tnear, const vfloat4& tfar, vfloat4& tNear, size_t& mask)  {

    // if this is a leaf, no intersection
    if(current_node.isLeaf()) {
      return false;
    }
    else {
      // if this is a set leaf, remove the encoding and continue
      if( current_node.isSetLeaf() ) {
	SetNode* snode = (SetNode*)current_node.snode();
	current_node = snode->ref();
      }

      if(current_node.isAligned()) {
	// perform ray intersection
	mask = intersectBox(*(AANode*)current_node.anyNode(), vray, tnear, tfar, tNear);
      }
      else if(current_node.isUnaligned()) {
	mask = intersectBox(*(UANode*)current_node.anyNode(), vray, tnear, tfar, tNear);
      }
      
      if ( mask != 0 ) {
	nodes_visited++;
	size_t mask_copy = mask;

	while(mask_copy != 0) {
	  size_t r = __bscf(mask_copy);
	  // get the box for this set (should be the same for all children)
	      
	  if(current_node.isUnaligned()) {
	    
	    // retrieve bounding box for this leaf from the parent node
	    Vec3fa corners[8];
	    ((UANode*)current_node.anyNode())->get_corners(r, corners);
	    
	    // write box to class MOAB instance
	    corners_to_hex(corners);
	  }
	  else if(current_node.isAligned()) {
	    
	    // retrieve bounding box for this leaf from the parent node
	    AABB box = ((AANode*)current_node.anyNode())->getBound(r);
	    
	    // write box to class MOAB instance
	    aabb_to_hex(box);
	    
	  }
	  
	}

	std::stringstream outfilename;
	outfilename << "step_" << std::setfill('0') << std::setw(4) << nodes_visited << ".vtk";
	write_and_clear(outfilename.str());
      
	// if there is a mix of leafs and interior nodes, make sure the interior nodes
	// come last in the traversal by artificially setting distances
	for (size_t i = 0; i < N; i++) {
	  if ( !current_node.bnode()->child(i).isLeaf() ) tNear[i] = inf;
	}
      }
      
    }
    return true;
    
  }

  virtual void setLeaf(NodeRef current_node, const NodeRef& previous_node) {
    // nothing to do for set leaves
    return; 
  }

  virtual void leaf(NodeRef current_node, const NodeRef& previous_node, const NodeRef& last_set_leaf, Ray& ray) {
    // if node is empty, do nothing
    if ( current_node.isEmpty() ) return;
    
    nodes_visited++;

    int child_number = find_child_number(current_node, previous_node);

    if(child_number < 0) {
      child_number = find_child_number(last_set_leaf, previous_node);
    }
    
    if(previous_node.isUnaligned()) {
    
      // retrieve bounding box for this leaf from the parent node
      Vec3fa corners[8];
      previous_node.uanode()->get_corners(child_number, corners);

      // write box to class MOAB instance
      corners_to_hex(corners);

    }
    else if(previous_node.isAligned()) {
      
      // retrieve bounding box for this leaf from the parent node
      AABB box = ((AANode*)previous_node.anyNode())->getBound(child_number);

      // write box to class MOAB instance
      aabb_to_hex(box);
      
    }
    
    moab::ErrorCode rval;
    // get leaf entities
    size_t numPrims;
    MBTriangleRef* prims = (MBTriangleRef*)current_node.leaf(numPrims);
    
    for(size_t i = 0; i < numPrims; i++) {
      moab::EntityHandle tri = prims[i].eh;
      transfer_tri(tri);
    }

    std::stringstream outfilename;
    outfilename << "step_" << std::setfill('0') << std::setw(4) << nodes_visited << ".vtk";
    write_and_clear(outfilename.str());
    
    return;
  }
  
  int get_nodes_visited() { return nodes_visited; }
  
};

int main (int argc, char** argv) {

  // define program options
  ProgOptions po("A program for representing a single ray traversal as a VTK file database.");

  std::string filename;

  po.addRequiredArg<std::string>("DAGMC Model", "File name of the DAGMC model.", &filename);
  double x, y, z;
  po.addRequiredArg<double>("x", "x position of ray origin", &x);
  po.addRequiredArg<double>("y", "y position of ray origin", &y);
  po.addRequiredArg<double>("z", "z position of ray origin", &z);
  double u, v, w;
  po.addRequiredArg<double>("u", "u direction of ray direction", &u);
  po.addRequiredArg<double>("v", "v direction of ray direction", &v);
  po.addRequiredArg<double>("w", "w direction of ray direction", &w);

  double ray_length;
  po.addOpt<double>("r", "Create a file representing the ray (ray.vtk) for visualization. The user-specified value is the length of the ray.");
		    
  int vol_id = 1;
  po.addOpt<int>("i", "ID of the volume to write as hexes. (1 by default)", &vol_id);

  // parse command line
  po.parseCommandLine(argc, argv);

  // create the MOAB instance and load the file
  moab::Interface *MBI = new moab::Core();
  moab::ErrorCode rval;
  
  rval = MBI->load_file(filename.c_str());
  MB_CHK_SET_ERR(rval, "Failed to load the DAGMC model: " << filename);

  // setup the BVH manager
  MBVHManager *BVHManager = new MBVHManager(MBI);
  // build trees for all geometric sets
  rval = BVHManager->build_all();
  MB_CHK_SET_ERR(rval, "Failed to construct trees");

  //get the global id tag and the geometry dim tag
  moab::Tag gid_tag;
  rval = MBI->tag_get_handle(GLOBAL_ID_TAG_NAME, gid_tag);
  MB_CHK_SET_ERR(rval, "Failed to retrieve the global id tag");
  moab::Tag cat_tag;
  rval = MBI->tag_get_handle(CATEGORY_TAG_NAME, cat_tag);
  MB_CHK_SET_ERR(rval, "Failed to retrieve the category tag");
  
  // set data
  // set tag value based on user-provided option
  std::string cat_name;
  int ent_id;
  cat_name = "Volume";
  ent_id = vol_id;
  cat_name.resize(CATEGORY_TAG_SIZE);
  const void* ptr[2] = { &ent_id, cat_name.c_str()};
  moab::Tag tags[2] = {gid_tag, cat_tag};
  
  moab::Range entities;

  // get the entity of interest from the mesh database
  rval = MBI->get_entities_by_type_and_tag(0, moab::MBENTITYSET, tags, ptr, 2, entities);
  MB_CHK_SET_ERR(rval, "Failed to retrieve geom entity set");

  // check that we've found exactly one entity
  if(entities.size() > 1) {
    std::cout << "Too many entities found. Please check the model." << std::endl;
    return 1;
  }
  if(entities.size() == 0) {
    std::cout << "Could not find entity of interest. Please specify a volume id in the model." << std::endl;
    return 1;
  }

  // get the tree root
  moab::EntityHandle ent = entities[0];
  NodeRef root = *(BVHManager->get_root(ent));
  std::cout << "Writing tree for " << cat_name << " with id " << ent_id << " and handle " << entities[0] << std::endl;

  //create the traversal class
  BVHCustomTraversal* tool = new BVHCustomTraversal();

  // set up ray 
  MBRay ray;
  ray.tnear = 0.0;
  ray.tfar = inf;
  ray.instID = vol_id;
  ray.org = Vec3da(x,y,z);
  ray.dir = Vec3da(u,v,w);

  // perform traversal
  TravWriter* op = new TravWriter(MBI);
  // write the ray if requested
  if ( po.getOpt("r", &ray_length) ) {
  op->create_ray(ray, ray_length);
  }
  tool->traverse(root, ray, *op);

  // cleanup
  delete MBI;
  delete BVHManager;
  delete tool;
  
  return 0;
}

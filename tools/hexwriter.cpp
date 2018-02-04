

#include "MBVH.h"
#include "MBVHManager.h"

#include "moab/Core.hpp"
#include "moab/ProgOptions.hpp"

#include "TraversalClass.hpp"

#include <string>

typedef BVHCustomTraversalT<Vec3da, double, moab::EntityHandle> BVHCustomTraversal;

class HexWriter : public BVHOperator {

  virtual bool visit(NodeRef& current_node, size_t& mask, vfloat4& tnear) {
    mask = 15;
    tnear = 0.0f;
    if( current_node.isSetLeaf() ) { current_node = current_node.setLeaf(); }
    return !current_node.isLeaf();
  }

  virtual void setLeaf(NodeRef current_node) {
    return;
  }

  virtual void leaf(NodeRef current_node) {
    return;
  }
  
};

int main (int argc, char** argv) {

  // define program options
  ProgOptions po("A program for representing the MBVH as a series of hex mesh elements.");

  std::string filename;
  po.addRequiredArg<std::string>("DAGMC Model", "File name of the DAGMC model.", &filename);

  int vol_id = -1, surf_id = -1;
  po.addOpt<int>("v", "ID of the volume to write as hexes. (1 by default)", &vol_id);
  po.addOpt<int>("s", "ID of the surface to write as hexes.", &surf_id);

  // parse command line
  po.parseCommandLine(argc, argv);

  // check that options were used correctly
  if ( surf_id > 0 && vol_id > 0 ) {
    std::cout << "Both a volume and surface were specified. Please use either the --v or --s flag, not both." << std::endl;
    return 1;
  }
  if ( surf_id < 0 && vol_id < 0 ) {
    std::cout << "Please specify either a volume or surface to write." << std::endl;
    return 1;
  }
  
  
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
  if (surf_id > 0) {
    cat_name = "Surface";
    ent_id = surf_id;
  }

  if (vol_id > 0) {
    cat_name = "Volume";
    ent_id = vol_id;
  }
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
    std::cout << "Could not find entity of interest. Please specify a surface or volume id in the model." << std::endl;
    return 1;
  }

  // get the tree root
  moab::EntityHandle ent = entities[0];
  NodeRef root = *(BVHManager->get_root(ent));
  std::cout << "Writing tree for " << cat_name << " with id " << ent_id << " and handle " << entities[0] << std::endl;

  //create the traversal class
  BVHCustomTraversal*  tool = new BVHCustomTraversal();
  MBRay ray;
  HexWriter* op = new HexWriter();
  tool->traverse(root, ray, *op);
  
  return 0;
  
}

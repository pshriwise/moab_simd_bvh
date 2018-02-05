

#include "MBVH.h"
#include "MBVHManager.h"

#include "moab/Core.hpp"
#include "moab/ProgOptions.hpp"

#include "TraversalClass.hpp"

#include <string>

typedef BVHCustomTraversalT<Vec3da, double, moab::EntityHandle> BVHCustomTraversal;

class HexWriter : public BVHOperator {

public:
  HexWriter() {
    num_leaves = 0;

    hw_mbi = new moab::Core();
    
  }
  
private:
  int num_leaves;
  moab::Interface *hw_mbi;
  moab::Range leaf_set;
  
public:
  
  virtual bool visit(NodeRef& current_node, TravRay vray, const vfloat4& tnear, const vfloat4& tfar, vfloat4& tNear, size_t& mask)  {
    // if this is a leaf, no intersection
    if(current_node.isLeaf()) {
      return false;
    }
    else {

      // if this is not a leaf, visit all child nodes
      mask = 15;

      // if this is a set leaf, remove the encoding and continue
      if( current_node.isSetLeaf() ) { current_node = current_node.setLeaf(); }

      // if there is a mix of leafs and interior nodes, make sure the interior nodes
      // come last in the traversal by artificially setting distances
      tNear = 100.0f;
      for (size_t i = 0; i < N; i++) {
	if ( current_node.bnode()->child(i).isLeaf() ) tNear[i] = 0.0f;
      }

    }
    return true;
    
  }

  virtual void setLeaf(NodeRef current_node) {
    return;
  }

  virtual void leaf(NodeRef current_node, NodeRef previous_node) {
    // if node is empty, do nothing
    
    if ( current_node.isEmpty() ) return;
    
    num_leaves++;
    
    const Node* node = previous_node.bnode();
    
    // find what "number" this child is
    size_t child_number = -1;
    for(size_t i = 0; i < N; i++){
      if ( node->child(i) == current_node ) {
	child_number = i;
	break;
      }
    }
    assert( child_number >= 0 );

    // retrieve bounding box for this leaf from the parent node
    AABB box = previous_node.node()->getBound(child_number);

    // create vertex coordinates for hex element
    std::vector<double> vertex_coords;
    // lower face in Z
    vertex_coords.push_back(box.lower[0]); vertex_coords.push_back(box.lower[1]); vertex_coords.push_back(box.lower[2]);
    vertex_coords.push_back(box.upper[0]); vertex_coords.push_back(box.lower[1]); vertex_coords.push_back(box.lower[2]);
    vertex_coords.push_back(box.upper[0]); vertex_coords.push_back(box.upper[1]); vertex_coords.push_back(box.lower[2]);
    vertex_coords.push_back(box.lower[0]); vertex_coords.push_back(box.upper[1]); vertex_coords.push_back(box.lower[2]);
    // upper face in Z
    vertex_coords.push_back(box.lower[0]); vertex_coords.push_back(box.lower[1]); vertex_coords.push_back(box.upper[2]);
    vertex_coords.push_back(box.upper[0]); vertex_coords.push_back(box.lower[1]); vertex_coords.push_back(box.upper[2]);
    vertex_coords.push_back(box.upper[0]); vertex_coords.push_back(box.upper[1]); vertex_coords.push_back(box.upper[2]);
    vertex_coords.push_back(box.lower[0]); vertex_coords.push_back(box.upper[1]); vertex_coords.push_back(box.upper[2]);

    // create mesh vertices and hex element for box
    moab::ErrorCode rval;
    moab::Range hex_verts;
    rval = hw_mbi->create_vertices(&(vertex_coords[0]), 8, hex_verts);
    MB_CHK_ERR_CONT(rval);

    // convet to vector - MOAB has no element constructor using Ranges?
    std::vector<moab::EntityHandle> hex_vert_vec;
    for(moab::Range::iterator i = hex_verts.begin(); i != hex_verts.end() ; i++) {
      hex_vert_vec.push_back(*i);
    }

    // create hex element
    moab::EntityHandle hex;
    rval = hw_mbi->create_element(moab::MBHEX, &(hex_vert_vec[0]), 8, hex);
    MB_CHK_ERR_CONT(rval);
        
    return;
  }

  AABB get_child_box(Node node, size_t child_number) {
    
  }
  
  int get_num_leaves() { return num_leaves; }

  void write() {
    moab::ErrorCode rval;
    rval = hw_mbi->write_file("leaf_boxes.h5m");
    MB_CHK_ERR_CONT(rval);
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
  MBRay ray; ray.tfar = inf;
  HexWriter* op = new HexWriter();
  tool->traverse(root, ray, *op);
  op->write();

  std::cout << "Num leaves found: " << op->get_num_leaves() << std::endl;
  
  return 0;
  
}

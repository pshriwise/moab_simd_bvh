

#include "MBVH.h"
#include "MBVHManager.h"

#include "moab/Core.hpp"
#include "moab/ProgOptions.hpp"

#include "TraversalClass.hpp"
#include "WriteVisitor.hpp"
#include <string>
#include <sstream>
#include <iomanip>

typedef BVHCustomTraversalT<Vec3da, double, moab::EntityHandle> BVHCustomTraversal;

class LeafWriter : public WriteVisitor {

public:

  LeafWriter(moab::Interface* original_moab_instance,
	    bool write_tris = true,
	    bool write_leaves = true,
	    bool write_set_leaves = false) : WriteVisitor(original_moab_instance),
					     num_leaves(0),
					     num_set_leaves(0),
					     write_leaves(write_leaves),
					     write_set_leaves(write_set_leaves),
					     write_tris(write_tris)
  {
    found_tris.clear();
  }

  ~LeafWriter() {
    delete new_mbi;
  }
  
private:
  bool write_tris, write_leaves, write_set_leaves;
  
  // some counters
  int num_leaves;
  int num_set_leaves;
  int num_leaf_triangles_written;
  int num_set_leaf_triangles_written;

  moab::Range found_tris;
  
public:
  
  virtual bool visit(NodeRef current_node, TravRay& vray,
		     const vfloat4& tnear,
		     const vfloat4& tfar,
		     vfloat4& tNear,
		     size_t& mask)  {    
    // if this is a leaf, no intersection
    if(current_node.isLeaf() || current_node.isSetLeaf() ) {
      return false;
    }
    else {
      // if this is not a leaf, visit all child nodes
      mask = 15;
      
      // if there is a mix of leafs and interior nodes, make sure the interior nodes
      // come last in the traversal by artificially setting distances
      tNear = 100.0f;
      for (size_t i = 0; i < N; i++) {
	if ( current_node.bnode()->child(i).isLeaf() ) tNear[i] = 0.0f;
	if ( current_node.bnode()->child(i).isSetLeaf() ) tNear[i] = 10.0f;
      }
    }
    
    return true;
  }

  virtual void setLeaf(NodeRef current_node) {

    num_set_leaves++;

    // if hexes for set leaves weren't requested, then do nothing
    if (!write_set_leaves) { return; }

    SetNodeT<moab::EntityHandle>* snode = (SetNodeT<moab::EntityHandle>*)current_node.snode();
    current_node = snode->ref();

    if(current_node.isUnaligned()) {
    
      // retrieve bounding box for this leaf from the parent node
      Vec3fa corners[8];
      current_node.uanode()->get_corners(0, corners);

      // write box to class MOAB instance
      corners_to_hex(corners);

    }
    else if(current_node.isAligned()) {
      
      // retrieve bounding box for this leaf from the parent node
      AABB box = ((AANode*)current_node.anyNode())->bounds();
      // write box to class MOAB instance
      aabb_to_hex(box);
      
    }

    moab::EntityHandle surface_handle = snode->setID;

    // if requrested, gather all triangles for this surface and and write
    // them as well
    if(write_tris) {
      moab::ErrorCode rval;
      moab::Range tris;
      rval = orig_mbi->get_entities_by_type(surface_handle, moab::MBTRI, tris);
      MB_CHK_ERR_CONT(rval);

      transfer_tris(tris);
      num_set_leaf_triangles_written += tris.size();
    }

    std::stringstream outfilename;
    outfilename << "set_leaf_box_" << std::setfill('0') << std::setw(4) << num_set_leaves << ".vtk";
    write_and_clear(outfilename.str());
    
    return;
  }

  virtual void leaf(NodeRef current_node, const NodeRef& previous_node, const NodeRef& last_set_leaf, Ray& ray) {

    // if node is empty, do nothing
    if ( current_node.isEmpty() ) { return; }

    num_leaves++;
    
    // if no leaf writing was requested, then do nothing
    if (!write_leaves) { return; }
    
    // get the child number of this node using the parent node
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

    // if triangles are to be written with the hexes, then
    // decode the leaf and transfer the triangle to the
    // class MOAB instance
    if(write_tris) {
      // get leaf entities
      size_t numPrims;
      MBTriangleRef* prims = (MBTriangleRef*)current_node.leaf(numPrims);
      
      for(size_t i = 0; i < numPrims; i++) {
	moab::EntityHandle tri = prims[i].eh;
	found_tris.insert(tri);
	transfer_tri(tri);
	num_leaf_triangles_written++;
      }
    }

    // create the filename using the number of leaves visited
    std::stringstream outfilename;
    outfilename << "leaf_box_" << std::setfill('0') << std::setw(4) << num_leaves << ".vtk";
    write_and_clear(outfilename.str());
    
    return;
  }
  
  int get_num_leaves() { return num_leaves; }

  int get_num_set_leaves() { return num_set_leaves; }

  int get_num_leaf_triangles_written() { return num_leaf_triangles_written; }
  
  int get_num_set_leaf_triangles_written() { return num_set_leaf_triangles_written; }

  bool validate_tree(moab::EntityHandle ent, bool verbose = true) {
    moab::ErrorCode rval;
    moab::Range ent_triangles;
    rval = orig_mbi->get_entities_by_type(ent, moab::MBTRI, ent_triangles, true);
    MB_CHK_ERR_CONT(rval);
    
    moab::Range result = subtract(ent_triangles, found_tris);
    
    if(!result.empty() && verbose) {
      std::cout << "Warning: Tree does not contain all entities underneath the specified entity set." << std::endl;
      std::cout << "Triangles in the set: " << ent_triangles.size() << std::endl;
      std::cout << "Triangles found in the traversal: " << found_tris.size() << std::endl;
      std::cout << "Number missing: " << result.size() << std::endl;
      std::cout << "Triangles missing from the tree" << std::endl;
      std::cout << result << std::endl;
    }
    
    return result.empty();
  }
  
};

int main (int argc, char** argv) {

  // define program options
  ProgOptions po("A program for representing the leaves of the MBVH as a series of VTK database files.");

  std::string filename;
  po.addRequiredArg<std::string>("DAGMC Model", "File name of the DAGMC model.", &filename);

  int vol_id = -1, surf_id = -1;
  bool write_set_leaves = false, write_tris = false;
  po.addOpt<int>("v", "ID of the volume to write as hexes. (1 by default)", &vol_id);
  po.addOpt<int>("s", "ID of the surface to write as hexes.", &surf_id);
  po.addOpt<void>("write-sets", "Write box files for the set (surface) nodes.", &write_set_leaves);
  po.addOpt<void>("write-tris", "Write triangles along with the leaf boxes.", &write_tris);

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
  LeafWriter* op = new LeafWriter(MBI, write_tris, true, write_set_leaves);
  tool->traverse(root, ray, *op);

  // output about standard leaves
  std::cout << "Num leaves found: " << op->get_num_leaves() << std::endl;
  if(write_tris) std::cout << "Num leaf triangles written: " << op->get_num_leaf_triangles_written() << std::endl;
  
  std::cout << std::endl;

  // output about set leaves
  if (write_set_leaves) {
    std::cout << "Num set leaves found: " << op->get_num_set_leaves() << std::endl;
    if(write_tris) std::cout << "Num set leaf triangles written: " << op->get_num_set_leaf_triangles_written() << std::endl;
  }

  // cleanup
  delete MBI;
  delete tool;
  delete BVHManager;
  
  return 0;
  
}

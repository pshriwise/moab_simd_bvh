

#include "MBVH.h"
#include "MBVHManager.h"

#include "moab/Core.hpp"
#include "moab/ProgOptions.hpp"

#include "BaseVisitor.hpp"
#include "TraversalClass.hpp"
#include <string>
#include <sstream>
#include <iomanip>


typedef BVHCustomTraversalT<Vec3da, double, moab::EntityHandle> BVHCustomTraversal;

class ValidationVisitor : public BaseVisitor {

public:
  
  ValidationVisitor(moab::Interface* original_moab_instance, moab::EntityHandle ent_set) : BaseVisitor(original_moab_instance) {
    ent_set = ent_set;
  }
  
  int num_leaves;
  int num_set_leaves;
  int num_nodes;

  moab::EntityHandle ent_set;
  moab::Range tris_found;
  
  virtual bool visit(NodeRef current_node, TravRay& vray,
		     const vfloat4& tnear,
		     const vfloat4& tfar,
		     vfloat4& tNear,
		     size_t& mask)  {
        // if this is a leaf, no intersection
    if(current_node.isLeaf()) {
      return false;
    }
    else {
      // if this is not a leaf, visit all child nodes
      mask = 15;

      // if this is a set leaf, remove the encoding and continue
      if( current_node.isSetLeaf() ) {
	setLeaf(current_node);
	current_node = current_node.setLeaf();
      }

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
    num_set_leaves++;
    return;
  }
  
  virtual void leaf(NodeRef current_node, const NodeRef& previous_node, const NodeRef& set_parent, Ray& ray) {
        // if node is empty, do nothing
    if ( current_node.isEmpty() ) { return; }

    num_leaves++;

    // get the child number of this node using the parent node
    int child_number = find_child_number(current_node, previous_node);

        // retrieve bounding box for this leaf from the parent node
    AABB box = previous_node.node()->getBound(child_number);

    size_t numPrims;
    MBTriangleRef* prims = (MBTriangleRef*)current_node.leaf(numPrims);

    moab::ErrorCode rval;
    
    for(size_t i = 0; i < numPrims; i++) {
      moab::EntityHandle tri = prims[i].eh;

      // add triangle to range
      tris_found.insert(tri);

      // make sure all triangle vertices are within its box     
      moab::Range verts;
      rval = original_mbi()->get_connectivity(&tri, 1, verts);
      MB_CHK_ERR_CONT(rval);

      Vec3da coords;
      moab::EntityHandle vert;
      vert = verts[0];
      rval = original_mbi()->get_coords(&vert, 1, &(coords.x));
      MB_CHK_ERR_CONT(rval);
      assert(inside(box, coords));

      vert = verts[1];
      rval = original_mbi()->get_coords(&vert, 1, &(coords.x));
      MB_CHK_ERR_CONT(rval);
      assert(inside(box, coords));
      
      vert = verts[2];
      rval = original_mbi()->get_coords(&vert, 1, &(coords.x));
      MB_CHK_ERR_CONT(rval);
      assert(inside(box, coords));

    }

    return;
  }
          
  bool validate() {
    moab::ErrorCode rval;
    moab::Range ent_triangles;
    rval = orig_mbi->get_entities_by_type(ent_set, moab::MBTRI, ent_triangles, true);
    MB_CHK_ERR_CONT(rval);
    
    moab::Range result = subtract(ent_triangles, tris_found);
    
    if(!result.empty()) {
      std::cout << "Warning: Tree does not contain all entities underneath the specified entity set." << std::endl;
      std::cout << "Triangles in the set: " << ent_triangles.size() << std::endl;
      std::cout << "Triangles found in the traversal: " << tris_found.size() << std::endl;
      std::cout << "Number missing: " << result.size() << std::endl;
      std::cout << "Triangles missing from the tree" << std::endl;
      std::cout << result << std::endl;
    }

    return result.empty();
  }

};


int main(int argc, char** argv) {

  // define program options
  ProgOptions po("A program for validating the trees of a model.");

  std::string filename;
  po.addRequiredArg<std::string>("DAGMC Model", "File name of the DAGMC model.", &filename);


  po.addOpt<int>("v", "ID of the volume to write as hexes. (1 by default)");
  po.addOpt<int>("s", "ID of the surface to write as hexes.");

  // parse command line
  po.parseCommandLine(argc, argv);
    
  // create the MOAB instance and load the file
  moab::Interface *MBI = new moab::Core();
  moab::ErrorCode rval;
  
  rval = MBI->load_file(filename.c_str());
  MB_CHK_SET_ERR(rval, "Failed to load the DAGMC model: " << filename);
  
  moab::Range sets_to_validate;
  int vol_id = -1, surf_id = -1;
  if (po.getOpt("v", &vol_id) || po.getOpt("s", &surf_id) ) {

    // check that options were used correctly
    if ( surf_id > 0 && vol_id > 0 ) {
      std::cout << "Both a volume and surface were specified. Please use either the --v or --s flag, not both." << std::endl;
      return 1;
    }

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
  
    // get the entity of interest from the mesh database
    rval = MBI->get_entities_by_type_and_tag(0, moab::MBENTITYSET, tags, ptr, 2, sets_to_validate);
    MB_CHK_SET_ERR(rval, "Failed to retrieve geom entity set");

    // check that we've found exactly one entity
    if(sets_to_validate.size() > 1) {
      std::cout << "Too many sets_to_validate found. Please check the model." << std::endl;
      return 1;
    }
    if(sets_to_validate.size() == 0) {
      std::cout << "Could not find entity of interest. Please specify a surface or volume id in the model." << std::endl;
      return 1;
    }
  }
  else {

    moab::Tag geom_tag;
    rval = MBI->tag_get_handle(GEOM_DIMENSION_TAG_NAME, geom_tag);
    MB_CHK_SET_ERR(rval, "Failed to retrieve the geometry dimension tag");

    int dim = 3;
    void* ptr = &dim;
    moab::Range vols;    
    // get the entity of interest from the mesh database
    rval = MBI->get_entities_by_type_and_tag(0, moab::MBENTITYSET, &geom_tag, &ptr, 1, vols);
    MB_CHK_SET_ERR(rval, "Failed to retrieve geom entity set");

    moab::Range surfs;
    dim = 2;
    rval = MBI->get_entities_by_type_and_tag(0, moab::MBENTITYSET, &geom_tag, &ptr, 1, surfs);
    MB_CHK_SET_ERR(rval, "Failed to retrieve geom entity set");

    sets_to_validate.merge(vols);
    sets_to_validate.merge(surfs);
  }
    
  // setup the BVH manager
  MBVHManager *BVHManager = new MBVHManager(MBI);
  // build trees for all geometric sets
  rval = BVHManager->build_all();
  MB_CHK_SET_ERR(rval, "Failed to construct trees");

  if (sets_to_validate.empty()) {
    std::cout << "No geometric sets were found in the model." << std::endl;
    return 1;
  }

  std::cout << "Validating " << sets_to_validate.size() << " specified geometric meshsets..." << std::endl;
  int num_fails = 0;
  for(moab::Range::iterator i = sets_to_validate.begin() ; i != sets_to_validate.end(); i++) {
    
    // get the tree root
    moab::EntityHandle ent = *i;
    NodeRef root = *(BVHManager->get_root(ent));
    
    //create the traversal class
    BVHCustomTraversal*  tool = new BVHCustomTraversal();
    MBRay ray; ray.tfar = inf;
    ValidationVisitor* op = new ValidationVisitor(MBI, ent);
    tool->traverse(root, ray, *op);
    if(!op->validate()) {
      std::cout << "Validation failed for entity with handle " << *i << std::endl;
      num_fails++;
    }
    
    delete tool;
    delete op;
    
  }

  if(num_fails) {
    std::cout << "There were " << num_fails << " trees which failed to validate." << std::endl;
  }
  else {
    std::cout << "Done." << std::endl;
    std::cout << "All trees are considered valid by the tool." << std::endl;
  }
  
  delete MBI;
  delete BVHManager;
  
  return 0;
}

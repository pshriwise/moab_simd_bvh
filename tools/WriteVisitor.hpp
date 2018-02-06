

#include "BaseVisitor.hpp"
#include "moab/Core.hpp"

class WriteVisitor : public BVHOperator<Vec3da,double,moab::EntityHandle> {

public:
  WriteVisitor(moab::Interface* mbi) {
    orig_mbi = mbi;
    new_mbi = new moab::Core();
  }
  
  // MOAB instance used to write vtk files
  moab::Interface* new_mbi;
  // MOAB instance used to load file and build the tree
  moab::Interface* orig_mbi;

  ~WriteVisitor() { delete new_mbi; }
  
public:

  moab::Interface* write_mbi() { return new_mbi; }

  moab::Interface* original_mbi() { return orig_mbi; }
  void write_and_clear(std::string filename) {
    
    moab::ErrorCode rval;
    rval = write_mbi()->write_file(filename.c_str());
    MB_CHK_ERR_CONT(rval);

    // clean out mesh for next leaf write
    rval = write_mbi()->delete_mesh();
    MB_CHK_ERR_CONT(rval);

    return;
  }
  
  moab::EntityHandle aabb_to_hex(AABB box) {
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
    rval = write_mbi()->create_vertices(&(vertex_coords[0]), 8, hex_verts);
    MB_CHK_ERR_CONT(rval);

    // convet to vector - MOAB has no element constructor using Ranges?
    std::vector<moab::EntityHandle> hex_vert_vec;
    for(moab::Range::iterator i = hex_verts.begin(); i != hex_verts.end() ; i++) {
      hex_vert_vec.push_back(*i);
    }

    // create hex element
    moab::EntityHandle hex;
    rval = write_mbi()->create_element(moab::MBHEX, &(hex_vert_vec[0]), 8, hex);
    MB_CHK_ERR_CONT(rval);

    return hex;
  }
  
  int find_child_number(NodeRef current_node, NodeRef previous_node) {
    
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

    return child_number;
  }

  std::vector<moab::EntityHandle> transfer_tris(std::vector<moab::EntityHandle> tris) {
    std::vector<moab::EntityHandle> new_tris;
    for(std::vector<moab::EntityHandle>::iterator i = tris.begin(); i != tris.end(); i++) {
      new_tris.push_back(transfer_tri(*i));
    }
    return new_tris;
  }
  
  moab::Range transfer_tris(moab::Range tris) {
    moab::Range new_tris;
    for(moab::Range::iterator i = tris.begin(); i != tris.end(); i++) {
      new_tris.insert(transfer_tri(*i));
    }
    return new_tris;
  }
  
  moab::EntityHandle transfer_tri(moab::EntityHandle tri) {
    moab::ErrorCode rval;
    // get vertices from original instance
    moab::Range verts;
    rval = original_mbi()->get_connectivity(&tri, 1, verts);
    MB_CHK_ERR_CONT(rval);
    assert(verts.size() == 3);

    // get the triangle's vertex coordinates
    moab::CartVect coords[3];
    rval = original_mbi()->get_coords(verts, coords[0].array());
    MB_CHK_ERR_CONT(rval);

    // create vertices in the other instance
    moab::Range new_verts;
    rval = write_mbi()->create_vertices(coords[0].array(), 3, new_verts);
    MB_CHK_ERR_CONT(rval);

    std::vector<moab::EntityHandle> new_verts_vec;
    for(moab::Range::iterator i = new_verts.begin(); i != new_verts.end(); i++) {
      new_verts_vec.push_back(*i);
    }
    // create triangle
    moab::EntityHandle new_tri;
    rval = write_mbi()->create_element(moab::MBTRI, &(new_verts_vec[0]), 3, new_tri);
    MB_CHK_ERR_CONT(rval);

    return new_tri;
  }


  void create_ray(Ray ray, double length) {
    moab::ErrorCode rval;
    moab::EntityHandle ray_verts[2];

    double coords1[3] = {ray.org.x, ray.org.y, ray.org.z};
    rval = write_mbi()->create_vertex(coords1, ray_verts[0]);
    MB_CHK_ERR_CONT(rval);

    Vec3da end_pnt = ray.org + ray.dir * length;
    double coords2[3] = {end_pnt.x, end_pnt.y, end_pnt.z};
    rval = write_mbi()->create_vertex(coords2, ray_verts[1]);
    MB_CHK_ERR_CONT(rval);

    
    moab::EntityHandle edge;
    rval = write_mbi()->create_element(moab::MBEDGE, &(ray_verts[0]), 2, edge);
    MB_CHK_ERR_CONT(rval);

    write_and_clear("ray.vtk");
    
    return;
  }
  

};



#include "Node.h"
#include "Vec3.h"
#include <vector>


size_t generate_tree(int current_depth, int split_axis, AABB box, int depth, std::vector<AANode>& nodes);

int main(int argc, char** argv) {

  AABB bbox(Vec3f(0.0, 0.0, 0.0),Vec3f(5.0, 5.0, 5.0));
  std::vector<AANode> nodes;
  int result = generate_tree(0, 0, bbox, 3, nodes);
  std::cout << "Num_nodes: " << nodes.size() << std::endl;
  
  return 0;

}

/*
 A function which will artificially generate a Quad-Tree
 intended for testing the traversal of the resulting tree.

 inputs: depth, and bounds
 output: root node of resulting tree

 A tree of with the specified depth will be created by splitting the initial set
 of bounds evenly along the x, then y, then z dimension as new levels of the
 tree are created. This continues until a tree of the specified depth is
 obtained. The nodes at this level are then declared leaf nodes and the root
 node of the tree is returned.
 */
size_t generate_tree(int current_depth, int split_axis, AABB box, int depth, std::vector<AANode>& nodes) {


  
  Vec3f dxdydz = (box.upper - box.lower)/4.0f;
  
  int node_idx = 0;
  if(current_depth <= depth) {
    // std::cout << "Here" << std::endl;
    // std::cout << "Split axis: " << split_axis << std::endl;
    // std::cout << "Delta " << dxdydz << std::endl;

    //create new child bounds
    vfloat4 bounds[6];

    bounds[0] = vfloat4(box.lower[0]); // lower x
    bounds[1] = vfloat4(box.upper[0]); // upper x
    bounds[2] = vfloat4(box.lower[1]); // lower y
    bounds[3] = vfloat4(box.upper[1]); // upper y
    bounds[4] = vfloat4(box.lower[2]); // lower z
    bounds[5] = vfloat4(box.upper[2]); // upper z

    float lb = box.lower[split_axis];
    float delta = dxdydz[split_axis];
    bounds[2*split_axis] = vfloat4(lb, lb + delta, lb + 2*delta, lb + 3*delta);
    bounds[2*split_axis+1] = vfloat4(lb + delta, lb + 2*delta, lb + 3*delta, lb + 4*delta);

    // set the node bounds
    AANode this_node = AANode(bounds[0], bounds[1], bounds[2], bounds[3], bounds[4], bounds[5]);
    node_idx = nodes.size();
    nodes.push_back(this_node);

    //generate children
    int new_split_axis = split_axis == 2 ? 0 : split_axis+1;    
    current_depth++;

    // child zero
    AABB box0 = AABB(Vec3f(bounds[0][0],bounds[2][0],bounds[4][0]),
		     Vec3f(bounds[1][0],bounds[3][0],bounds[5][0]));
    this_node.children[0] = NodeRef(generate_tree(current_depth, new_split_axis, box0, depth, nodes));

    // child one
    AABB box1 = AABB(Vec3f(bounds[0][1],bounds[2][1],bounds[4][1]),
		     Vec3f(bounds[1][1],bounds[3][1],bounds[5][1]));
    this_node.children[1] = NodeRef(generate_tree(current_depth, new_split_axis, box1, depth, nodes));

    // child two
    AABB box2 = AABB(Vec3f(bounds[0][2],bounds[2][2],bounds[4][2]),
		     Vec3f(bounds[1][2],bounds[3][2],bounds[5][2]));
    this_node.children[2] = NodeRef(generate_tree(current_depth, new_split_axis, box2, depth, nodes));

    // child three
    AABB box3 = AABB(Vec3f(bounds[0][3],bounds[2][3],bounds[4][3]),
		     Vec3f(bounds[1][3],bounds[3][3],bounds[5][3]));
    this_node.children[3] = NodeRef(generate_tree(current_depth, new_split_axis, box3, depth, nodes));

    // cast address of node to size_t and return
    return (size_t)&this_node;	      
  }
  else {
    return -1;
  }
}


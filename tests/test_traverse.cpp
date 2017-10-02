

#include "Node.h"
#include "Vec3.h"
#include "Traverser.h"
#include "Intersector.h"
#include "Ray.h"
#include "Vec3fa.h"
#include "Builder.h"

#include <vector>

#include "testutil.hpp"

size_t generate_tree(int current_depth, int split_axis, AABB box, int depth);
void print_tree(AANode root, int depth);

void test_intersect();

int main(int argc, char** argv) {

  test_intersect();
  
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
size_t generate_tree(int current_depth, int split_axis, AABB box, int depth) {
  
  
  
  if(current_depth <= depth) {
    // determine the child box interval in each dimension
    Vec3fa dxdydz = (box.upper - box.lower)/4.0f;

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
    AANode* this_node = new AANode(bounds[0], bounds[1], bounds[2], bounds[3], bounds[4], bounds[5]);

    //generate children
    int new_split_axis = split_axis == 2 ? 0 : split_axis+1;    
    current_depth++;
    
    // create child nodes/leaves
    for(unsigned int i=0; i < N; i++){
      AABB box = AABB(Vec3fa(bounds[0][i],bounds[2][i],bounds[4][i]),
		      Vec3fa(bounds[1][i],bounds[3][i],bounds[5][i]));
      this_node->children[i] = generate_tree(current_depth, new_split_axis, box, depth);
    }

    // return a reference to this node
    return NodeRef((size_t) this_node);	      
  }
  else {
    return NodeRef(tyLeaf);
  }
}

void print_tree(AANode node, int depth) {

  // print the depth
  std::cout << "Depth: " << depth << std::endl;
  // print the current node
  std::cout << node << std::endl;

  // print child nodes (if not leaves)
  for(unsigned int i = 0; i < 4; i++) {
    if (node.child(i) != tyLeaf) {
      AANode n = *node.child(i).node();
      std::cout << n << std::endl;
      print_tree(n,depth+1);
    }
  }
  
}

void test_intersect() {
  // create an initial box
  AABB bbox(Vec3fa(0.0, 0.0, 0.0),Vec3fa(4.0, 4.0, 4.0));

  // generate the tree for this box
  NodeRef root_ref = generate_tree(0, 0, bbox, 4);
  print_tree(*root_ref.node(),0);
  
  // create a ray for intersection with the hierarchy
  Vec3fa org(10.0, 2.5, 2.5), dir(-1.0, 0.0, 0.0);
  Ray r(org, dir);

  // use the root reference node to traverse the ray
  BuildPrimitiveIntersector BVH;
  BVH.intersectRay(root_ref, r);
}

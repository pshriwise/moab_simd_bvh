
#pragma once

#include <vector>
#include <list>
#include <set>
#include "Primitive.h"
#include "BuildState.h"
#include "TempNode.h"
#include "Node.h"

#define MAX_LEAF_SIZE 8

enum BuildQuality {
  BUILD_QUALITY_LOW = 0,
  BUILD_QUALITY_NORMAL = 1,
  BUILD_QUALITY_HIGH = 2
};
  

struct BBounds{
  float lower_x, lower_y, lower_z, align0;
  float upper_x, upper_y, upper_z, align1;
};

struct BuildSettings {
  unsigned size;
  BuildQuality quality;
  unsigned BranchingFactor;
  unsigned maxDepth;
  unsigned sahBlockSize;
  unsigned minLeafSize;
  unsigned maxLeafSize;
  float travCost;
  float intCost;
};


typedef TempNodeT <BuildPrimitive> TempNodeBP;

/// leaf encoding ///
// this function takes in a pointer to
// the location at which a set of primitive references are
// stored as part of the tree structure. Because primitives
// are required to take up 16 bytes, the number of
// primitives is stored in the THREE LEAST SIGNIFICANT bits of
// the ptr attribute in the returned node reference.
// The node is also marked as a leaf using the tyLeaf value
// which should not interfere with the other encoded bytes,
// being the FOURTH leas significant bit.
NodeRef* encodeLeaf(void *prim_arr, size_t num) {
  assert(num < MAX_LEAF_SIZE);
  return new NodeRef((size_t)prim_arr | (tyLeaf + num));
}

struct Heuristic {

  virtual void split(BuildPrimitive* primitives, size_t numPrimitives,
		     BuildPrimitive* rightPrimitives, size_t numRightPrimitives,
		     BuildPrimitive* leftPrimitives, size_t numLeftPrimitives) = 0;

  AABB bounds(BuildPrimitive* primitives, size_t numPrimitives) {
    AABB b((float)inf,(float)neg_inf);
    for(size_t i=0; i<numPrimitives; i++) {
      b.update(primitives[i].lower.x,primitives[i].lower.y,primitives[i].lower.z);
      b.update(primitives[i].upper.x,primitives[i].upper.y,primitives[i].upper.z);

    }
    return b;
  }

};

typedef void* (*createLeafFunc) (BuildPrimitive* primitives, size_t numPrimitives);

void createNode(NodeRef* node) {
  AANode* ptr = new AANode();
  node = new NodeRef((size_t)ptr);
}


void sortPrimitives(const int dim, const float coord,
		    const BuildPrimitive* primitives, const size_t numPrimitives,
		    BuildPrimitive* leftPrimitives, size_t& numLeftPrimitives,
		    BuildPrimitive* rightPrimitives, size_t& numRightPrimitives) {

  std::vector<BuildPrimitive> left_list, right_list;
  
  for(size_t i=0; i<numPrimitives; i++){
    if( primitives[i].center()[dim] - coord > 0.0 ){
      left_list.push_back(primitives[i]);
    }
    else {
      right_list.push_back(primitives[i]);
    }
  }

  leftPrimitives = &(left_list[0]);
  numLeftPrimitives = (size_t) left_list.size();
  rightPrimitives = &(right_list[0]);
  numRightPrimitives = (size_t) right_list.size();
}


void* create_leaf(BuildPrimitive* primitives, size_t numPrimitives) {
  return (void*) encodeLeaf((void*)primitives, numPrimitives - 1);
  }


template <typename T> class BVHBuilder {

 public:
  inline BVHBuilder(createLeafFunc createLeaf = NULL) : maxLeafSize(8), depth(0), maxDepth(BVH_MAX_DEPTH), largest_leaf_size(0), smallest_leaf_size(maxLeafSize), numLeaves(0) {}
  
 private:
  size_t maxLeafSize;
  size_t depth;
  size_t maxDepth;
  size_t largest_leaf_size, smallest_leaf_size;
  size_t numLeaves;

  std::list< BuildStateT<T> > storage;
 public:


  void* createLeaf(T* primitives, size_t numPrimitives) {
  return (void*) encodeLeaf((void*)primitives, numPrimitives - 1);
  }

  void splitNode(NodeRef* node, size_t split_axis, const T* primitives, const size_t numPrimitives, TempNodeT<T> tn[N]) {

    assert(split_axis >= 0 && split_axis <= 2);

    //get the bounds of the node
    AABB box((float)inf,(float)neg_inf);
    for(size_t i=0; i<numPrimitives; i++) {
      box.update(primitives[i].lower.x,primitives[i].lower.y,primitives[i].lower.z);
      box.update(primitives[i].upper.x,primitives[i].upper.y,primitives[i].upper.z);
    }

    Vec3fa dxdydz = (box.upper - box.lower) / 4.0f;

    //create new child bounds
    vfloat4 bounds[6];
  
    new (&bounds[0]) vfloat4(box.lower[0]); // lower x
    new (&bounds[1]) vfloat4(box.upper[0]); // upper x
    new (&bounds[2]) vfloat4(box.lower[1]); // lower y
    new (&bounds[3]) vfloat4(box.upper[1]); // upper y
    new (&bounds[4]) vfloat4(box.lower[2]); // lower z
    new (&bounds[5]) vfloat4(box.upper[2]); // upper z

    float lb = box.lower[split_axis];
    float delta = dxdydz[split_axis];
    bounds[2*split_axis] = vfloat4(lb, lb + delta, lb + 2*delta, lb + 3*delta);
    bounds[2*split_axis+1] = vfloat4(lb + delta, lb + 2*delta, lb + 3*delta, lb + 4*delta);

    AABB boxes[N];

	 
    new (&boxes[0]) AABB(bounds[0][0],
		    bounds[2][0],
		    bounds[4][0],
		    bounds[1][0],
		    bounds[3][0],
		    bounds[5][0]);
    new (&boxes[1]) AABB(bounds[0][1],
		    bounds[2][1],
		    bounds[4][1],
		    bounds[1][1],
		    bounds[3][1],
		    bounds[5][1]);
    new (&boxes[2]) AABB(bounds[0][2],
		    bounds[2][2],
		    bounds[4][2],
		    bounds[1][2],
		    bounds[3][2],
		    bounds[5][2]);
    new (&boxes[3]) AABB(bounds[0][3],
		    bounds[2][3],
		    bounds[4][3],
		    bounds[1][3],
		    bounds[3][3],
		    bounds[5][3]);

    tn[0].clear();
    tn[1].clear();
    tn[2].clear();
    tn[3].clear();

    /* new (&tn[0]) TempNodeT<T>(); */
    /* new (&tn[1]) TempNodeT<T>(); */
    /* new (&tn[2]) TempNodeT<T>(); */
    /* new (&tn[3]) TempNodeT<T>(); */
  
    // sort primitives into boxes by their centroid
    for(size_t i = 0; i < numPrimitives; i++) {
      const T* p = &(primitives[i]);
      bool placed = false;
      for(size_t j = 0; j < 4 ; j++) {
	// if the centroid is in the box, place it there
	if( inside(boxes[j], p->center()) ){
	  placed = true;
	  tn[j].prims.push_back(*p);
	  tn[j].box.update(p->lower.x, p->lower.y, p->lower.z);
	  tn[j].box.update(p->upper.x, p->upper.y, p->upper.z);
	  break;
	}
      }
      assert(placed);
    }

  }


  void splitNode(NodeRef* node, const T* primitives, const size_t numPrimitives, TempNodeT<T> tempNodes[N]) {

    // split node along each axis
    float max_cost = 2.0;
    float min_cost = 0.0;
  
    float best_cost = max_cost;
    int best_dim = -1;
    float cost;

    AANode* this_node = node->node();

    AABB node_box = this_node->bounds();
  
    // split along each axis and get lowest cost
    for(size_t i = 0; i < 3; i++) {
      splitNode(node, i, primitives, numPrimitives, tempNodes);
      //compute the SAH cost of this node split
      cost = tempNodes[0].sah_contribution() +
	tempNodes[1].sah_contribution() +
	tempNodes[2].sah_contribution() +
	tempNodes[3].sah_contribution();
      cost /= area(node_box)*(float)numPrimitives;

      assert(cost >= min_cost && cost <= max_cost);
      // update cost
      if (cost < best_cost) {
	best_cost = cost;
	best_dim = i;
      }
    }

    assert(best_dim != -1);

    splitNode(node, best_dim, primitives, numPrimitives, tempNodes);

    vfloat4 low_x, upp_x,
      low_y, upp_y,
      low_z, upp_z;

    new (&low_x) vfloat4(tempNodes[0].box.lower.x,
			 tempNodes[1].box.lower.x,
			 tempNodes[2].box.lower.x,
			 tempNodes[3].box.lower.x);
    new (&low_y) vfloat4(tempNodes[0].box.lower.y,
			 tempNodes[1].box.lower.y,
			 tempNodes[2].box.lower.y,
			 tempNodes[3].box.lower.y);
    new (&low_z) vfloat4(tempNodes[0].box.lower.z,
			 tempNodes[1].box.lower.z,
			 tempNodes[2].box.lower.z,
			 tempNodes[3].box.lower.z);
    new (&upp_x) vfloat4(tempNodes[0].box.upper.x,
			 tempNodes[1].box.upper.x,
			 tempNodes[2].box.upper.x,
			 tempNodes[3].box.upper.x);
    new (&upp_y) vfloat4(tempNodes[0].box.upper.y,
			 tempNodes[1].box.upper.y,
			 tempNodes[2].box.upper.y,
			 tempNodes[3].box.upper.y);
    new (&upp_z) vfloat4(tempNodes[0].box.upper.z,
			 tempNodes[1].box.upper.z,
			 tempNodes[2].box.upper.z,
			 tempNodes[3].box.upper.z);

    float bump = 5e-03;
    this_node->set(low_x - bump, upp_x + bump,
		   low_y - bump, upp_y + bump,
		   low_z - bump, upp_z + bump);
    
    return;
  }

  
  void stats () { std::cout << "Depth: " << depth << std::endl;
    std::cout << "Largest leaf: " << largest_leaf_size << std::endl;
    std::cout <<  "Smallest leaf: " << smallest_leaf_size << std::endl;
    std::cout << "Number of leaves: " << numLeaves << std::endl;  
  }

  void splitFallback(const BuildStateT<T>& current, BuildStateT<T>& left, BuildStateT<T>& right) {
    const size_t begin_id = current.prims.prims.front().primID();
    const size_t end_id = current.prims.prims.back().primID();
    const size_t center_id = (begin_id + end_id)/2;

    size_t numPrimitives = current.prims.size();
    for (size_t i = 0; i < current.prims.size(); i++) {
      if (current.prims[i].primID() < center_id) {
	left.prims.push_back(current.prims[i]);
      }
      else {
	right.prims.push_back(current.prims[i]);
      }

    }

      return;
  }
  
  NodeRef* createLargeLeaf(BuildStateT<T>& current) {
    
    /* if(current.depth > maxDepth) { */
    /*   std::cerr << "Maximum depth reached" << std::endl; */
    /*   std::cerr << "Current depth: " << current.depth << std::endl; */
    /*   std::cerr << "Maximum allowed depth: " << maxDepth << std::endl; */
    /*   std::cerr << "Number of primitives remaining: " << current.size() << std::endl; */
    /*   assert(false); */
    /* } */

    if (current.size() <= maxLeafSize) {
      if (current.size() > largest_leaf_size) largest_leaf_size = current.size();
      if (current.size() < smallest_leaf_size) smallest_leaf_size = current.size();
      numLeaves++;
      depth = current.depth > depth ? current.depth : depth;
      storage.push_back(current);
      return storage.back().size() ? (NodeRef*) createLeaf(storage.back().ptr(), storage.back().size()) : new NodeRef();
    }

    
    BuildStateT<T> tempChildren[N];
    size_t numChildren = 1;
    AABB bounds = current.prims.bounds();
    tempChildren[0] = current;
    for( size_t i = 1; i < numChildren; i++) {
      new (&tempChildren[i]) BuildStateT<T>(current.depth+1);
    }
    
    do {

      size_t best_child = -1;
      size_t best_size = 0;
      for (size_t i = 0; i < numChildren; i++) {
	/* ignore leaf if under the limit */
	if(tempChildren[i].prims.size() <= maxLeafSize)
	  continue;

	/* track child with largest size */
	if (tempChildren[i].prims.size() > best_size) {
	  best_size = tempChildren[i].prims.size();
	  best_child = i;
	}
      }

      /* if no child over maxLeafSize, then we're done */
      if(best_child == (size_t)-1) break;

      BuildStateT<T> left(current.depth+1);
      BuildStateT<T> right(current.depth+1);
      /* split the best child into left and right */
      splitFallback(tempChildren[best_child], left, right);

      /* add new children */
      tempChildren[best_child] = tempChildren[numChildren-1];
      tempChildren[numChildren-1] = left;
      tempChildren[numChildren+0]= right;
      numChildren++;
      
      /* find child with largest bounding box area */
    } while (numChildren < N);

    /* get children bounds */
    vfloat4 x_min, y_min, z_min, x_max, y_max, z_max;
    
    for (size_t i = 0; i < numChildren; i++) {
      AABB b;
      BuildSetT<T> primitives = tempChildren[i].prims;
      
      for(size_t j = 0; j < primitives.size(); j++) {
	b.update(primitives[j].lower.x, primitives[j].lower.y, primitives[j].lower.z);
	b.update(primitives[j].upper.x, primitives[j].upper.y, primitives[j].upper.z);
      }
      
      x_min[i] = b.lower.x; y_min[i] = b.lower.y; z_min[i] = b.lower.z;
      x_max[i] = b.upper.x; y_max[i] = b.upper.y; z_max[i] = b.upper.z;
      
    }
    
    /* create node */
    AANode* aanode = new AANode(x_min, x_max, y_min, y_max, z_min, z_max);
    NodeRef* node = new NodeRef((size_t)aanode);

    
    /* recurse into each child and perform reduction */
    for (size_t i = 0; i < numChildren; i++) {
#ifdef VERBOSE_MODE
      std::cout << "Recurring into CLL" << std::endl;
      std::cout << "Sending " << tempChildren[i].size() << " primitives" << std::endl;
      std::cout << *aanode << std::endl;
#endif
      NodeRef* child_node = createLargeLeaf(tempChildren[i]);
      aanode->setRef(i, *child_node);
    }

    depth = current.depth > depth ? current.depth : depth;
    
    return node;

  }

  NodeRef* Build(BuildStateT<T>& current,
		 const BuildSettings& settings = BuildSettings()
		 //	    createNodeFunc createNode,
		 //	    linkChildrenFunc linkChildren,
		 //	    setNodeBoundsFunc setNodeBounds,
		 ) {


    const T* primitives = current.ptr();
    size_t numPrimitives = current.size();
    
    // if the end conditions for the tree are met, then create a leaf
    if(numPrimitives <= maxLeafSize || current.depth > maxDepth) {
#ifdef VERBOSE_MODE
      std::cout << "Sending " << current.size() << std::endl;
#endif
      return createLargeLeaf(current);
    }


    // created a new node and set the bounds
    AANode* aanode = new AANode();
    AABB box;
    new (&box) AABB((float)inf, (float)neg_inf);
    for(size_t i = 0; i < numPrimitives; i++) {
      box.update(primitives[i].lower.x, primitives[i].lower.y, primitives[i].lower.z);
      box.update(primitives[i].upper.x, primitives[i].upper.y, primitives[i].upper.z);
    }

    // increment depth and recur here
    aanode->setBounds(box);
    NodeRef* this_node = new NodeRef((size_t)aanode);
    TempNodeT<T> tempNodes[4];
    splitNode(this_node, primitives, numPrimitives, tempNodes);

#ifdef VERBOSE_MODE
    std::cout << "New Node with Bounds: " << std::endl << *aanode << std::endl;
    std::cout << "At depth: " << depth << std::endl;
#endif

    for(size_t i = 0; i < N ; i++){
      BuildStateT<T>* br = new BuildStateT<T>(current.depth+1, tempNodes[i].prims);
      NodeRef* child_node = Build(*br);
      // link the child node
      aanode->setRef(i, *child_node);
    }

    depth = current.depth > depth ? current.depth : depth;
    
    return this_node;
  } // end build

};

typedef BVHBuilder<BuildPrimitive> BuildPrimitiveBVH;


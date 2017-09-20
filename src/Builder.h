
#include <vector>
#include <set>
#include "Primitive.h"

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


NodeRef* encodeLeaf(void *prim_arr, size_t num) {
  //  assert(numPrimitives < MAX_LEAF_SIZE); needs to be re-added later
  return new NodeRef((size_t)prim_arr | (tyLeaf + num));
}

struct Heuristic {

  virtual void split(BuildPrimitive* primitives, size_t numPrimitives,
		     BuildPrimitive* rightPrimitives, size_t numRightPrimitives,
		     BuildPrimitive* leftPrimitives, size_t numLeftPrimitives) = 0;

  AABB bounds(BuildPrimitive* primitives, size_t numPrimitives) {
    AABB b((float)inf,(float)neg_inf);
    for(size_t i=0; i<numPrimitives; i++) {
      b.update(primitives[i].lower_x,primitives[i].lower_y,primitives[i].lower_z);
      b.update(primitives[i].upper_x,primitives[i].upper_y,primitives[i].upper_z);

    }
    return b;
  }

};



typedef void* (*createNodeFunc) (size_t numChildren);

typedef void (*linkChildrenFunc) (void* nodePtr, void** children, size_t numChildren);

typedef void (*setNodeBoundsFunc) (void* nodePtr, const BBounds** bounds, size_t numChildren);

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



struct TempNode {
  AABB box;
  std::vector<BuildPrimitive> prims;

  inline TempNode (AABB& b, std::vector<BuildPrimitive>& p) : box(b), prims(p) {}
  
  inline TempNode() : box(AABB((float)inf,(float)neg_inf)) {}

  void update_box() {
    for (size_t i = 0; i < prims.size() ; i++) {
      box.update(prims[i].lower_x,prims[i].lower_y,prims[i].lower_z);
      box.update(prims[i].upper_x,prims[i].upper_y,prims[i].upper_z);
    }
  }
  
  float sah_contribution() { return (0 == prims.size()) ? 0.0 : area(box)*(float)prims.size(); }
};

template<typename T>
struct SetT{
  
  inline SetT(std::set<T>& p) { std::vector<T> v(prims.begin(), prims.end());
                                  prims = v;
                              }

  inline SetT(const std::vector<T>& p) : prims(p) {}
  
  inline SetT() {}

  inline SetT(EmptyTy) {}

  inline AABB bounds() {
    AABB box;
    for(size_t i = 0; i < prims.size(); i++ ) {
      box.update(prims[i].lower_x,prims[i].lower_y,prims[i].lower_z);
      box.update(prims[i].upper_x,prims[i].upper_y,prims[i].upper_z);
    }
  }

  inline void clear() { prims.clear(); }

  inline size_t size() const { prims.size(); }

  inline T* ptr () { return &(*prims.begin()); }

  inline const T& operator []( const size_t index) const { assert(index < prims.size()); return prims[index];}
  inline       T& operator []( const size_t index)       { assert(index < prims.size()); return prims[index]; }

  inline void push_back(const T& p) { prims.push_back(p); }
  
  std::vector<T> prims;
};

typedef SetT<BuildPrimitive> Set;

template<typename T>
struct BuildRecordT {
public:
  BuildRecordT () {}

  BuildRecordT (size_t depth) : depth(depth) { prims.clear(); }

  BuildRecordT (size_t depth, const std::vector<T> &p)
  : depth(depth) { prims = SetT<T>(p);
                 }

  BuildRecordT (const std::vector<T> &prims)
  : depth(0), prims(prims) {}

  BuildRecordT (size_t depth, T* primitives, size_t numPrimitives)
  : depth(depth) { std::vector<T> s(primitives, primitives + numPrimitives);
                   prims = SetT<T>(s);
                 }

  BuildRecordT (T* primitives, size_t numPrimitives)
  : depth(0) { std::vector<T> s(primitives, primitives + numPrimitives);
               prims = SetT<T>(s);
             }

  friend bool  operator< (const BuildRecordT& a, const BuildRecordT& b) { return a.prims.size() < b.prims.size(); }

  friend bool operator> (const BuildRecordT& a, const BuildRecordT& b) { return a.prims.size() > b.prims.size(); }

  size_t size() const { return prims.size(); }

  inline T* ptr () { return prims.ptr(); }

public:
  size_t depth;
  
  SetT<T> prims;
};

typedef BuildRecordT<BuildPrimitive> BuildRecord;


void splitNode(NodeRef* node, size_t split_axis, const BuildPrimitive* primitives, const size_t numPrimitives, TempNode tn[N]) {

  assert(split_axis >= 0 && split_axis <= 2);

  //get the bounds of the node
  AABB box((float)inf,(float)neg_inf);
  for(size_t i=0; i<numPrimitives; i++) {
    box.update(primitives[i].lower_x,primitives[i].lower_y,primitives[i].lower_z);
    box.update(primitives[i].upper_x,primitives[i].upper_y,primitives[i].upper_z);

  }

  Vec3fa dxdydz = (box.upper - box.lower) / 4.0f;

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

  AABB boxes[N];

	 
  boxes[0] = AABB(bounds[0][0],
		   bounds[2][0],
		   bounds[4][0],
		   bounds[1][0],
		   bounds[3][0],
		   bounds[5][0]);
  boxes[1] = AABB(bounds[0][1],
		   bounds[2][1],
		   bounds[4][1],
		   bounds[1][1],
		   bounds[3][1],
		   bounds[5][1]);
  boxes[2] = AABB(bounds[0][2],
		   bounds[2][2],
		   bounds[4][2],
		   bounds[1][2],
		   bounds[3][2],
		   bounds[5][2]);
  boxes[3] = AABB(bounds[0][3],
		   bounds[2][3],
		   bounds[4][3],
		   bounds[1][3],
		   bounds[3][3],
		   bounds[5][3]);

  tn[0] = TempNode();
  tn[1] = TempNode();
  tn[2] = TempNode();
  tn[3] = TempNode();
  
  // sort primitives into boxes by their centroid
  for(size_t i = 0; i < numPrimitives; i++) {
    BuildPrimitive p = primitives[i];
    bool placed = false;
    for(size_t j = 0; j < 4 ; j++) {
      // if the centroid is in the box, place it there
      if( inside(boxes[j], p.center()) ){
	placed = true;
	tn[j].prims.push_back(p);
	tn[j].box.update(p.lower_x, p.lower_y, p.lower_z);
	tn[j].box.update(p.upper_x, p.upper_y, p.upper_z);
      }
    }
    assert(placed);
  }

}


void splitNode(NodeRef* node, const BuildPrimitive* primitives, const size_t numPrimitives, TempNode tempNodes[N]) {

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

  low_x = vfloat4(tempNodes[0].box.lower.x,
		  tempNodes[1].box.lower.x,
		  tempNodes[2].box.lower.x,
		  tempNodes[3].box.lower.x);
  low_y = vfloat4(tempNodes[0].box.lower.y,
		  tempNodes[1].box.lower.y,
		  tempNodes[2].box.lower.y,
		  tempNodes[3].box.lower.y);
  low_z = vfloat4(tempNodes[0].box.lower.z,
		  tempNodes[1].box.lower.z,
		  tempNodes[2].box.lower.z,
		  tempNodes[3].box.lower.z);
  upp_x = vfloat4(tempNodes[0].box.upper.x,
		  tempNodes[1].box.upper.x,
		  tempNodes[2].box.upper.x,
		  tempNodes[3].box.upper.x);
  upp_y = vfloat4(tempNodes[0].box.upper.y,
		  tempNodes[1].box.upper.y,
		  tempNodes[2].box.upper.y,
		  tempNodes[3].box.upper.y);
  upp_z = vfloat4(tempNodes[0].box.upper.z,
		  tempNodes[1].box.upper.z,
		  tempNodes[2].box.upper.z,
		  tempNodes[3].box.upper.z);

  this_node->set(low_x,upp_x,
		 low_y,upp_y,
		 low_z,upp_z);
  return;
  
}

void* create_leaf(BuildPrimitive *primitives, size_t numPrimitives) {
  
  return (void*) encodeLeaf((void*)primitives, numPrimitives);
}

struct PrimRef{
  inline PrimRef () {}

  inline PrimRef (const AABB& bounds, unsigned int geomID, unsigned int primID)
  {
    lower = bounds.lower; upper.a = geomID;
    upper = bounds.upper; upper.a = primID;
  }

  inline const Vec3fa center2() const {
    return lower+upper;
  }

  inline const AABB bounds() const {
    return AABB(lower,upper);
  }

  inline unsigned size() const {
    return 1;
  }

  inline unsigned geomID() const {
    return lower.a;
  }

  inline unsigned primID() const {
    return upper.a;
  }

  public:
    Vec3fa lower, upper;
};



class BVHBuilder {

 public:
  inline BVHBuilder(createLeafFunc createLeaf) : maxLeafSize(8), depth(0), maxDepth(1024), createLeaf(createLeaf), largest_leaf_size(0), smallest_leaf_size(maxLeafSize), numLeaves(0) {}
  
 private:
  size_t maxLeafSize;
  size_t depth;
  size_t maxDepth;
  createLeafFunc createLeaf;
  size_t largest_leaf_size, smallest_leaf_size;
  size_t numLeaves;

  std::vector<BuildRecord> records;
  
 public:

  void stats () { std::cout << "Depth: " << depth << std::endl;
    std::cout << "Largest leaf: " << largest_leaf_size << std::endl;
    std::cout <<  "Smallest leaf: " << smallest_leaf_size << std::endl;
    std::cout << "Number of leaves: " << numLeaves << std::endl;  
  }

  void splitFallback(const BuildRecord& current, BuildRecord& left, BuildRecord& right) {
    const size_t begin_id = current.prims.prims.front().primID;
    const size_t end_id = current.prims.prims.back().primID;
    const size_t center_id = (begin_id + end_id)/2;

    size_t numPrimitives = current.prims.size();
    for (size_t i = 0; i < current.prims.size(); i++) {
      if (current.prims[i].primID < center_id) {
	left.prims.push_back(current.prims[i]);
      }
      else {
	right.prims.push_back(current.prims[i]);
      }

    }

      return;
  }
  
  NodeRef* createLargeLeaf(BuildRecord& current) {
    
    if(current.depth > maxDepth) {
      std::cerr << "Maximum depth reached" << std::endl;
      std::cerr << "Current depth: " << current.depth << std::endl;
      std::cerr << "Maximum allowed depth: " << maxDepth << std::endl;
      std::cerr << "Number of primitives remaining: " << current.size() << std::endl;
      assert(false);
    }

    if (current.size() <= maxLeafSize) {
      if (current.size() > largest_leaf_size) largest_leaf_size = current.size();
      if (current.size() < smallest_leaf_size) smallest_leaf_size = current.size();
      numLeaves++;
      records.push_back(current);
      return (NodeRef*) createLeaf(current.ptr(), current.size());
    }


    BuildRecord tempChildren[N];
    size_t numChildren = 1;
    AABB bounds = current.prims.bounds();
    tempChildren[0] = current;
    for( size_t i = 1; i < numChildren; i++) {
      tempChildren[i] = BuildRecord(current.depth+1);
    }
    
    do {

      size_t best_child = -1;
      size_t best_size = 0;
      for (size_t i = 0; i < numChildren; i++) {
	/* ignore leaf if under the limit */
	if(tempChildren[i].prims.size() < maxLeafSize)
	  continue;

	/* track child with largest size */
	if (tempChildren[i].prims.size() > best_size) {
	  best_size = tempChildren[i].prims.size();
	  best_child = i;
	}
      }

      /* if no child over maxLeafSize, then we're done */
      if(best_child == (size_t)-1) break;

      BuildRecord left(current.depth+1);
      BuildRecord right(current.depth+1);
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
      AABB b = tempChildren[i].prims.bounds();
      x_min[i] = b.lower.x; y_min[i] = b.lower.y; z_min[i] = b.lower.z;
      x_max[i] = b.upper.x; y_max[i] = b.upper.y; z_max[i] = b.upper.z;
      
    }
    
    /* create node */
    AANode* aanode = new AANode(x_min, y_min, z_min, x_max, y_max, z_max);

    NodeRef* node = new NodeRef((size_t)aanode);

    /* recurse into each child and perform reduction */
    for (size_t i = 0; i < numChildren; i++) {
      NodeRef* child_node = createLargeLeaf(tempChildren[i]);
      aanode->setRef(i, *child_node);
    }

    return node;

  }

  NodeRef* Build(const BuildSettings& settings,
		 BuildRecord& current
		 //	    createNodeFunc createNode,
		 //	    linkChildrenFunc linkChildren,
		 //	    setNodeBoundsFunc setNodeBounds,
		 ) {


    const BuildPrimitive* primitives = current.ptr();
    size_t numPrimitives = current.size();
    
    // if the end conditions for the tree are met, then create a leaf
    if(numPrimitives <= maxLeafSize || current.depth > maxDepth-20) {
      return createLargeLeaf(current);
    }


    // created a new node and set the bounds
    AANode* aanode = new AANode();
    AABB box((float)inf, (float)neg_inf);
    for(size_t i = 0; i < numPrimitives; i++) {
      box.update(primitives[i].lower_x,primitives[i].lower_y,primitives[i].lower_z);
      box.update(primitives[i].upper_x,primitives[i].upper_y,primitives[i].upper_z);
    }

    // increment depth and recur here
    depth++;
    aanode->setBounds(box);
    NodeRef* this_node = new NodeRef((size_t)aanode);
    TempNode tempNodes[4];
    splitNode(this_node, primitives, numPrimitives, tempNodes);
    
    for(size_t i = 0; i < N ; i++){
      BuildRecord br(current.depth+1, tempNodes[i].prims);
      NodeRef* child_node = Build(settings, br);
      // link the child node
      aanode->setRef(i, *child_node);
    }

    return this_node;
  } // end build

};

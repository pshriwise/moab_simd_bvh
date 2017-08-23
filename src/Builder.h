
#include <vector>
#include <set>


enum BuildQuality {
  BUILD_QUALITY_LOW = 0,
  BUILD_QUALITY_NORMAL = 1,
  BUILD_QUALITY_HIGH = 2
};
  
struct BuildPrimitive {
  float lower_x, lower_y, lower_z;
  float upper_x, upper_y, upper_z;

  Vec3fa center() const { return (Vec3fa(lower_x,lower_y,lower_z)+Vec3fa(upper_x,upper_y,upper_z))/2.0f; }
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


struct Heuristic {

  virtual void split(BuildPrimitive* primitives, size_t numPrimitives,
		     BuildPrimitive* rightPrimitives, size_t numRightPrimitives,
		     BuildPrimitive* leftPrimitives, size_t numLeftPrimitives) = 0;

  AABB bounds(BuildPrimitive* primitives, size_t numPrimitives) {
    AABB b;
    for(size_t i=0; i<numPrimitives; i++) {
      b.extend((AABB&)primitives[i]);
    }
    return b;
  }

};



typedef void* (*createNodeFunc) (size_t numChildren);

typedef void (*linkChildrenFunc) (void* nodePtr, void** children, size_t numChildren);

typedef void (*setNodeBoundsFunc) (void* nodePtr, const BBounds** bounds, size_t numChildren);

typedef void* (*createLeafFunc) (const BuildPrimitive* primitives, size_t numPrimitives);


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






void splitNode(NodeRef* node, size_t split_axis, const BuildPrimitive* primitives, const size_t numPrimitives) {

  assert(split_axis >= 0 && split_axis <= 2);

  //get the bounds of the node
  AABB box;
  for(size_t i=0; i<numPrimitives; i++) {
    box.extend((AABB&)primitives[i]);
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

  AABB boxes[4];
  boxes[0] = AABB(bounds[0][0], bounds[2][0], bounds[4][0], bounds[1][0], bounds[3][0], bounds[5][0]);
  boxes[1] = AABB(bounds[0][1], bounds[2][1], bounds[4][1], bounds[1][1], bounds[3][1], bounds[5][1]);
  boxes[2] = AABB(bounds[0][2], bounds[2][2], bounds[4][2], bounds[1][2], bounds[3][2], bounds[5][2]);
  boxes[3] = AABB(bounds[0][3], bounds[2][3], bounds[4][3], bounds[1][3], bounds[3][3], bounds[5][3]);


  // sort primitives into boxes by their centroid
  for(size_t i = 0; i < numPrimitives; i++) {
    BuildPrimitive p = primitives[i];
    bool placed = false;
    for(size_t j = 0; j < 4 ; j++) {
      // if the centroid is in the box, place it there
      if( inside(boxes[j], p.center()) ){
	placed = true;
      }
    }
    assert(placed);
  }
}

struct TempNode {
  AABB box;
  std::vector<BuildPrimitive> prims;

  float sah_contribution() { return area(box)*(float)prims.size(); }
};

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

struct Set{
  
  inline Set(std::set<PrimRef>& prims) : prims(prims) {}

  inline Set() {}

  inline Set(EmptyTy) {}
			      
  std::set<PrimRef> prims;
};

struct BuildState
{
  
  inline BuildState() {}

  inline BuildState(size_t depth)
  : depth(depth), prims(empty) {}

  inline BuildState(size_t depth, Set primitives)
  : depth(depth), prims(primitives) {}

  Set prims;
  size_t depth;  
};
  

class BVHBuilder {

 private:
  std::vector<BuildPrimitive> primitive_vec;

 public:
  
  NodeRef Build(const BuildSettings& settings,
		 BuildPrimitive* primitives,
		 size_t numPrimitives,
		 //	    createNodeFunc createNode,
		 //	    linkChildrenFunc linkChildren,
		 //	    setNodeBoundsFunc setNodeBounds,
		 createLeafFunc createLeaf) {

    // create a vector for the primitives to reside in
    primitive_vec.resize(numPrimitives);
    BuildPrimitive* vptr = primitive_vec.data();
    vptr = primitives;

    
    
    
    
    

    NodeRef* node;
    createNode(node);
    
  } // end builder
};

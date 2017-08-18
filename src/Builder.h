
#include <vector>



enum BuildQuality {
  BUILD_QUALITY_LOW = 0,
  BUILD_QUALITY_NORMAL = 1,
  BUILD_QUALITY_HIGH = 2
};
  
struct BuildPrimitive {
  float lower_x, lower_y, lower_z;
  float upper_x, upper_y, upper_z;

  Vec3f center() const { return (Vec3f(lower_x,lower_y,lower_z)+Vec3f(upper_x,upper_y,upper_z))/2.0f; }
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


void splitNode(NodeRef* node, const size_t numChildren, const BuildPrimitive* primitives, const size_t numPrimitives){

  //get the bounds of the node
  AABB box;

  for(size_t i=0; i<numPrimitives; i++) {
    box.extend((AABB&)primitives[i]);
  }

  //split along each dimension and pick best split
  size_t num_planes = numChildren-1;
  float pval = 1.0f/(float)num_planes;
  std::vector<float> plane_values;
  
  for(int ax=0; ax<3; ax++) {
    plane_values.clear();
    //create plane coordinates
    for(size_t j=0; j<num_planes; j++) {
      plane_values.push_back(pval*(box.upper[ax]-box.lower[ax]));
    }
    //sort primitives
    BuildPrimitive *left_list, *right_list;
    size_t numLeft, numRight;
    sortPrimitives(ax,plane_values[0],primitives,numPrimitives,left_list,numLeft,right_list,numRight);
      
  }
  
}

  
NodeRef* Build(const BuildSettings& settings,
	    BuildPrimitive* primitives,
	    size_t numPrimitives,
	       //	    createNodeFunc createNode,
	       //	    linkChildrenFunc linkChildren,
	       //	    setNodeBoundsFunc setNodeBounds,
	    createLeafFunc createLeaf) {

  NodeRef* node;
  createNode(node);

  

}



#include <set>
#include <vector>

#include "Builder.h"
#include "BuildState.h"
#include "Intersector.h"
#include "TriangleRef.h"
#include "MOABDirectAccessManager.h"

typedef BVHBuilder<TriangleRef> TriangleBVH;

typedef BuildStateT<TriangleRef> BuildStateTri;

typedef BVHIntersectorT<TriangleRef, Vec3fa, float> TriIntersector;

typedef BVHIntersectorT<TriangleRef, Vec3da, double> DblTriIntersector;

struct PrimRef{
  
  inline PrimRef () {}

  inline PrimRef( const Vec3fa& lower, const Vec3fa& upper, void* p, int i) : lower(lower), upper(upper), primitivePtr(p) { this->upper.a = i; }
  
  inline PrimRef (const AABB& bounds, unsigned int geomID, unsigned int primID)
  {
    lower = bounds.lower; upper.a = geomID;
    upper = bounds.upper; upper.a = primID;
  }
  
  inline const Vec3fa center() const {
    return (lower+upper)/2.0f;
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
  void* primitivePtr;
  
};


typedef BuildStateT<PrimRef> MBBuildState;
typedef TempNode<PrimRef> MBTempNode;

template <typename T>
class BVH {

 public:
  inline BVH(double* xPtr, double* yPtr, double* zPtr, void* primitivePtr, int numPrimitives, int stride, long unsigned int id) : numPrimitives(numPrimitives), vpere(stride), maxLeafSize(7), depth(0), maxDepth(BVH_MAX_DEPTH), largest_leaf_size(0), smallest_leaf_size(maxLeafSize), numLeaves(0), num_stored(0)
    {
      MDAM = new MOABDirectAccessManager(id, xPtr, yPtr, zPtr, primitivePtr);
      
      leaf_sequence_storage.resize(numPrimitives);
    }
     
 private:
  
  size_t maxLeafSize;
  size_t depth;
  size_t maxDepth;
  size_t largest_leaf_size, smallest_leaf_size;
  size_t numLeaves;

  int num_stored;

  std::vector<T> leaf_sequence_storage;

  MOABDirectAccessManager* MDAM;
  
  int numPrimitives;
  int vpere;

  static const size_t stackSize = 1+(N-1)*BVH_MAX_DEPTH;

 public:
  
  inline void* createLeaf(T* primitives, size_t numPrimitives) {
    return (void*) encodeLeaf((void*)primitives, numPrimitives - 1);
  }

  inline NodeRef* Build() {
    // create BuildState of PrimitiveReferences
    MBBuildState bs(0);
    for( size_t i = 0; i < numPrimitives; i++ ) {
      
      T triref = T((moab::EntityHandle*)MDAM->conn + (i*vpere), MDAM->id);

      Vec3fa lower, upper;
      
      triref.get_bounds(lower, upper, MDAM);
	
      PrimRef p(lower, upper, (void*)triref.eh, i);

      bs.prims.push_back(p);
    }

    return Build(bs);
  }

  inline NodeRef* Build(MBBuildState& current) {

    const PrimRef* primitives = current.ptr();
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
    MBTempNode tempNodes[4];
    splitNode(this_node, primitives, numPrimitives, tempNodes);

#ifdef VERBOSE_MODE
    std::cout << "New Node with Bounds: " << std::endl << *aanode << std::endl;
    std::cout << "At depth: " << depth << std::endl;
#endif

    for(size_t i = 0; i < N ; i++){
      MBBuildState* br = new MBBuildState(current.depth+1, tempNodes[i].prims);
      NodeRef* child_node = Build(*br);
      // link the child node
      aanode->setRef(i, *child_node);
    }

    depth = current.depth > depth ? current.depth : depth;
    
    return this_node;
  } // end build


  void splitNode(NodeRef* node, const PrimRef* primitives, const size_t numPrimitives, MBTempNode tempNodes[N]) {

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

    this_node->set(low_x,upp_x,
		   low_y,upp_y,
		   low_z,upp_z);
    return;
  
  }

  void splitNode(NodeRef* node, size_t split_axis, const PrimRef* primitives, const size_t numPrimitives, MBTempNode tn[N]) {

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

    /* new (&tn[0]) TempNode<T>(); */
    /* new (&tn[1]) TempNode<T>(); */
    /* new (&tn[2]) TempNode<T>(); */
    /* new (&tn[3]) TempNode<T>(); */
  
    // sort primitives into boxes by their centroid
    for(size_t i = 0; i < numPrimitives; i++) {
      const PrimRef* p = &(primitives[i]);
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

  NodeRef* createLargeLeaf(MBBuildState& current) {
    
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

      if(current.size() == 0 ) return new NodeRef();

      T* position = &(*(leaf_sequence_storage.begin()+num_stored));

      for( size_t i = 0; i < current.size(); i++) {
	
	T t = T((moab::EntityHandle*)MDAM->conn + (current.prims[i].primID()*vpere), MDAM->id);
	leaf_sequence_storage[num_stored+i] = t;
	
      }
      
      num_stored += (int)current.size();

      if ((size_t)position & 8 )  std::cout << "Uh-oh" << std::endl;
      
      return current.size() ? (NodeRef*) createLeaf(position, current.size()) : new NodeRef();
    }

    
    MBBuildState tempChildren[N];
    size_t numChildren = 1;
    AABB bounds = current.prims.bounds();
    tempChildren[0] = current;
    for( size_t i = 1; i < numChildren; i++) {
      new (&tempChildren[i]) MBBuildState(current.depth+1);
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

      MBBuildState left(current.depth+1);
      MBBuildState right(current.depth+1);
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
      BuildBuildSetT<PrimRef> primitives = tempChildren[i].prims;
      
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


  void splitFallback(const MBBuildState& current, MBBuildState& left, MBBuildState& right) {
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


  static inline bool intersect(NodeRef& node, const TravRay& ray, const vfloat4& tnear, const vfloat4& tfar, vfloat4& dist, size_t& mask) {
    if(node.isLeaf()) return false;
    mask = intersectBox(*node.node(),ray,tnear,tfar,dist);
    return true;
  }

  inline void intersectRay (NodeRef root, dRay & ray) {
    /* initialiez stack state */
    StackItemT<NodeRef> stack[stackSize];
    StackItemT<NodeRef>* stackPtr = stack+1;
    StackItemT<NodeRef>* stackEnd = stack+stackSize;
    stack[0].ptr = root;
    stack[0].dist = neg_inf;
    
    /* verify correct inputs */
    assert(ray.valid());
    assert(ray.tnear >= 0.0f);
    
    TravRay vray = TravRay(ray.org, ray.dir);
    vfloat4 ray_near = std::max(ray.tnear, 0.0);
    vfloat4 ray_far = std::max(ray.tfar, 0.0);
    
    BVHTraverser nodeTraverser;
    new (&nodeTraverser) BVHTraverser();
    
    TraversalTracker t;
    
    while (true) pop:
      {
	if(stackPtr == stack) break;
	stackPtr--;
	NodeRef cur = NodeRef(stackPtr->ptr);
	t.up();
	
	// if the ray doesn't reach this node, move to next
	if(*(float*)&stackPtr->dist > ray.tfar) { continue; }
        
	while (true)
	  {
	    size_t mask = 0; vfloat4 tNear(inf);
	    bool nodeIntersected = intersect(cur, vray, ray_near, ray_far, tNear, mask);
	    
#ifdef VERBOSE_MODE
	    AANode* curaa = cur.node();
	    if( !cur.isEmpty() ) std::cout << curaa->bounds() << std::endl;
	    else std::cout << "EMPTY NODE" << std::endl;
	    
	    if (nodeIntersected) {
	      std::cout << "INTERIOR NODE" << std::endl;
	      std::cout << std::bitset<4>(mask) << std::endl;
	      std::cout << "Distances to hit: " << tNear << std::endl;
	      std::cout << *cur.node() << std::endl;
	    }
	    else
	      std::cout << "LEAF NODE" << std::endl;
	    std::cout << std::endl;
#endif
	    // if no intersection, this is a leaf - check primitives
	    if (!nodeIntersected) {
	      // temporary setting of ray values
	      //	    ray.tnear = std::min(min(tNear),ray.tnear);
	      //	    ray.tfar = std::min(min(tNear),ray.tfar);
	      break; }
	    
	    t.down(mask);
	    // if no children were hit, pop next node
	    if (mask == 0) { goto pop; }
	    
	    nodeTraverser.traverseClosest(cur, mask, tNear, stackPtr, stackEnd);
	  }

	// leaf (set distance to nearest/farthest box intersection for now)
	size_t numPrims;
	T* primIDs = (T*)cur.leaf(numPrims);
	
	if ( !cur.isEmpty() ) {
	  for (size_t i = 0; i < numPrims; i++) {
	    T t = primIDs[i];
	    t.intersect(ray, (void*)MDAM);
	  }
	}
	
      }
    
    return;
  }

  void stats () { std::cout << "Depth: " << depth << std::endl;
    std::cout << "Largest leaf: " << largest_leaf_size << std::endl;
    std::cout <<  "Smallest leaf: " << smallest_leaf_size << std::endl;
    std::cout << "Number of leaves: " << numLeaves << std::endl;  
  }

  
};

typedef BVH<MBTriangleRef> MBVH;
typedef BVH<BuildPrimitive> BBVH;
typedef BVH<TriangleRef> TBVH;

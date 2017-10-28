


#pragma once

#include "Ray.h"
#include "Node.h"
#include "Intersector.h"
#include "Traverser.h"
#include "Stack.h"
#include "Primitive.h"

#include <vector>

// #define VERBOSE_MODE

#ifdef VERBOSE_MODE
  #include <bitset>
#endif

struct TraversalTracker {
  
  std::vector<int> s;

  inline TraversalTracker() { s.push_back(1); }

  inline void count_hits(size_t& mask, int& hits) {
    assert(mask <= 15 && mask >= 0);
    
    if(mask == 0) return;
    size_t r = __bscf(mask);
    hits++;
    count_hits(mask, hits);
    return;
  }
  
  inline void down(size_t mask) {
    assert(mask >= 0);

    s.back()--;

    if(mask == 0 ) return;
    
    int nodes_hit = 0;
    count_hits(mask, nodes_hit);
    s.push_back(nodes_hit);
    
    return;
  }
  
  inline void up() {

    if(s.back() == 0) {
      s.pop_back();
      up();
    }
    else{
      return;
    }
  }

  inline int depth() { return s.size()-1; }
  
};

  

template <typename T, typename V, typename P> class BVHIntersectorT {
  
  static const size_t stackSize = 1+(N-1)*BVH_MAX_DEPTH;

 public:
  //  void intersectRay(NodeRef root, Ray& ray);
  
  static inline bool intersect(NodeRef& node, const TravRay& ray, const vfloat4& tnear, const vfloat4& tfar, vfloat4& dist, size_t& mask) {
    if(node.isLeaf()) return false;
    mask = intersectBox(*node.node(),ray,tnear,tfar,dist);
    return true;
  }

  
  inline void intersectRay (NodeRef root, RayT<V,P> & ray) {

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
    vfloat4 ray_near = std::max(ray.tnear, (P)0.0);
    vfloat4 ray_far = std::max(ray.tfar, (P)0.0);

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
	T* prims = (T*)cur.leaf(numPrims);

	if ( !cur.isEmpty() ) {
	  for (size_t i = 0; i < numPrims; i++) {
	    T p = prims[i];
	    p.template intersect< RayT<V,P> >(ray);
	  }
	}

      }
    return;
  }

};

typedef BVHIntersectorT<BuildPrimitive, Vec3fa, float> BuildPrimitiveIntersector;

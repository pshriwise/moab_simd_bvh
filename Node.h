#pragma once

#include "Vec3.h"
#include "constants.h"
#include "vfloat.h"

#define N 4

static const size_t emptyNode = 8;
static const size_t tyLeaf = 8;

struct NodeRef {
  inline NodeRef () {}

  inline NodeRef(size_t ptr) : ptr(ptr) {}

  inline operator size_t() const { return ptr; }
  
  inline size_t isLeaf() const { return ptr & tyLeaf; }

  /* inline char* leaf(size_t& num) const { */
  /*   assert(isLeaf()); */
  /*   num = (ptr & (size_t)items_mask)-tyLeaf; */
  /*   return (char*)(ptr & ~(size_t)align_mask); */
  /* } */

private:
  size_t ptr;
		   

};

struct Node {
  inline Node () {}

  inline void clear() { for(size_t i=0; i < N; i++) children[i] = emptyNode; }

  inline bool verify() {
    for (size_t i=0; i < N; i++) {
      if (child(i) == emptyNode) {
	for(; i < N; i++) {
	  if (child(i) != emptyNode)
	    return false;
	}
	break;
      }
    }
    return true;
  }

  

  
  inline NodeRef& child(size_t i) { assert(i<N); return children[i]; }
  inline const NodeRef& child(size_t i) const { assert(i<N); return children[i]; }

  NodeRef children[N];
  
};



struct AANode : public Node
{

  using::Node::children;
  
  //  inline AANode& child(size_t i) { assert(i<N); return children[i]; }
  //inline const AANode& child(size_t i) const { assert(i<N); return children[i]; }

  // empty constructor
  inline AANode() {}


  inline void clear() { lower_x = lower_y = lower_z = neg_inf;
                        upper_x = upper_y = upper_z = inf;
			Node::clear();
                        }

  inline void setRef (size_t i, const NodeRef& ref) { assert(i<N); children[i] = ref; }
    

  inline void setBounds(const AABB& bounds) { lower_x = bounds.lower.x; lower_y = bounds.lower.y; lower_z = bounds.lower.z;
                                              upper_x = bounds.upper.x; upper_y = bounds.upper.y; upper_z = bounds.upper.z; }
  
  inline AABB bounds() const { const Vec3f lower(min(lower_x), min(lower_y),min(lower_z));
                               const Vec3f upper(max(upper_x), max(upper_y), max(upper_z));
  			       return AABB(lower, upper); }
  
  vfloat4 lower_x, upper_x, lower_y, upper_y, lower_z, upper_z;
  
};

  
inline size_t intersectBox(const AANode &node, const TravRay &ray, const float &tnear, const float &tfar, float &dist) {
  const float tNearX = (*(float*)((const char*)&node.lower_x + ray.nearX)- ray.org.x) * ray.rdir.x;
  const float tNearY = (*(float*)((const char*)&node.lower_x + ray.nearY) - ray.org.y) * ray.rdir.y;
  const float tNearZ = (*(float*)((const char*)&node.lower_x + ray.nearZ) - ray.org.z) * ray.rdir.z;
  const float tFarX = (*(float*)((const char*)&node.lower_x + ray.farX) - ray.org.x) * ray.rdir.x;
  const float tFarY = (*(float*)((const char*)&node.lower_x + ray.farY) - ray.org.y) * ray.rdir.y;
  const float tFarZ = (*(float*)((const char*)&node.lower_x + ray.farZ) - ray.org.z) * ray.rdir.z;

  const float round_down = 1.0f-2.0f*float(ulp); // FIXME: use per instruction rounding for AVX512
  const float round_up   = 1.0f+2.0f*float(ulp);

  const float tNear = std::max(std::max(tNearX, tNearY), tNearZ);
  const float tFar = std::min(std::min(tFarX, tFarY), tFarZ);
  const bool vmask = (round_down*tNear <= round_up*tFar);
  dist = tNear;
  return vmask;
};


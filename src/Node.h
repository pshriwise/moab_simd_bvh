#pragma once

#include "Vec3.h"
#include "AABB.h"
#include "constants.h"
#include "vfloat.h"
#include "Primitive.h"
#include "sys.h"
#include <immintrin.h>

static const size_t emptyNode = 8;
static const size_t tyLeaf = 8;
static const size_t setLeafAlign = 3;

static const size_t items_mask = 15;
static const size_t align_mask = 15;

// forward declarations
struct AANode;
struct Node;
 
struct NodeRef {

  __forceinline NodeRef () { ptr = emptyNode;}

  __forceinline NodeRef(size_t ptr) : ptr(ptr) {}

  __forceinline operator size_t() const { return ptr; }

  __forceinline size_t pointer () const { return ptr; }
  
  __forceinline size_t isLeaf() const { return ptr & tyLeaf; }

  __forceinline size_t isSetLeaf() const { return !(isLeaf()) && (ptr & setLeafAlign); }

  __forceinline bool isEmpty() const { return ptr == emptyNode; }

  __forceinline       AANode* node()       { return (AANode*)ptr; }
  __forceinline const AANode* node() const { return (const AANode*)ptr; }

  __forceinline       AANode* safeNode()        { return isSetLeaf() ? (AANode*)setLeafPtr() : node(); }
  __forceinline const AANode* safeNode() const  { return isSetLeaf() ? (AANode*)setLeafPtr() : node(); }
      
  __forceinline       void* snode()       { return (void*)setLeafPtr(); }
  __forceinline const void* snode() const { return (const void*)setLeafPtr(); }
  
  __forceinline       Node* bnode()       { return (Node*)ptr; }
  __forceinline const Node* bnode() const { return (const Node*)ptr; }

  __forceinline const void setPtr(size_t new_ptr) { ptr = new_ptr; }

  __forceinline size_t setLeafPtr() { return (ptr & ~(size_t)setLeafAlign); }
  __forceinline size_t setLeafPtr() const { return (ptr & ~(size_t)setLeafAlign); }

  __forceinline NodeRef setLeaf() { return NodeRef(setLeafPtr()); }
  
  __forceinline void* leaf(size_t& num) const {
    assert(isLeaf());
    num = 1 + (ptr & (items_mask))-tyLeaf;
    return (void*) (ptr & ~(size_t)align_mask);
  }

  __forceinline void prefetch() const {
    prefetchL2(((char*)ptr)+0*64);
    prefetchL2(((char*)ptr)+1*64);
    return;
  }
  
  /* __forceinline char* leaf(size_t& num) const { */
  /*   assert(isLeaf()); */
  /*   num = (ptr & (size_t)items_mask)-tyLeaf; */
  /*   return (char*)(ptr & ~(size_t)align_mask); */
  /* } */

  
  private:
  size_t ptr;
		   

};


__forceinline const bool operator ==( const NodeRef& a, const NodeRef& b ) { return a.pointer() == b.pointer(); }



struct Node {
  __forceinline Node () {}
  
  __forceinline void clear() { for(size_t i=0; i < N; i++) children[i] = emptyNode; }

  __forceinline bool verify() {
    for (size_t i=0; i < N; i++) {
      if (child(i) == NodeRef(emptyNode) ) {
	for(; i < N; i++) {
	  if (child(i) != NodeRef(emptyNode) )
	    return false;
	}
	break;
      }
    }
    return true;
  }
  
  __forceinline NodeRef& child(size_t i) { assert(i<N); return children[i]; }
  __forceinline const NodeRef& child(size_t i) const { assert(i<N); return children[i]; }

  NodeRef children[N];
  
};



struct __aligned(16) AANode : public Node
{

  using::Node::children;


  // __forceinline AANode& child(size_t i) { assert(i<N); return children[i]; }
  //  __forceinline const AANode& child(size_t i) const { assert(i<N); return children[i]; }

  // empty constructor
  __forceinline AANode() {}

  __forceinline AANode( const vfloat4& low_x, const vfloat4& up_x,
		 const vfloat4& low_y, const vfloat4& up_y,
		 const vfloat4& low_z, const vfloat4& up_z,
		 const NodeRef* child_ptr = NULL) : lower_x(low_x), upper_x(up_x),
                                                    lower_y(low_y), upper_y(up_y),
                                                    lower_z(low_z), upper_z(up_z) {
                                                    if (child_ptr) {
                                                      children[0] = *child_ptr;
					              children[1] = *(child_ptr+1);
					              children[2] = *(child_ptr+2);
					              children[3] = *(child_ptr+3);
						    }
                                                   }

  __forceinline void set( const vfloat4& low_x, const vfloat4& up_x,
		  const vfloat4& low_y, const vfloat4& up_y,
		  const vfloat4& low_z, const vfloat4& up_z,
		   const NodeRef* child_ptr = NULL) {
                                                      lower_x = low_x; upper_x = up_x;
                                                      lower_y = low_y; upper_y = up_y;
                                                      lower_z = low_z; upper_z = up_z;
                                                      if (child_ptr) {
                                                        children[0] = *child_ptr;
					                children[1] = *(child_ptr+1);
					                children[2] = *(child_ptr+2);
					                children[3] = *(child_ptr+3);
						      }
                                                   }

  __forceinline void clear() { lower_x = lower_y = lower_z = neg_inf;
                        upper_x = upper_y = upper_z = inf;
			Node::clear();
                        }

  __forceinline void setRef (size_t i, const NodeRef& ref) { assert(i<N); children[i] = ref; }

  __forceinline void setBound(size_t i, const AABB& bounds) { lower_x[i] = bounds.lower.x;
                                                       lower_y[i] = bounds.lower.y;
						       lower_z[i] = bounds.lower.z;
						       upper_x[i] = bounds.upper.x;
                                                       upper_y[i] = bounds.upper.y;
						       upper_z[i] = bounds.upper.z;  }
  
  inline AABB getBound(size_t i) { // check index value
                                   assert(i >= 0 && i < 4); 
                                   // create and return bounding box
                                   return AABB( lower_x[i], lower_y[i], lower_z[i],
						upper_x[i], upper_y[i], upper_z[i]); }

  __forceinline void setBounds(const AABB& bounds) { lower_x = bounds.lower.x; lower_y = bounds.lower.y; lower_z = bounds.lower.z;
                                              upper_x = bounds.upper.x; upper_y = bounds.upper.y; upper_z = bounds.upper.z; }
  
  __forceinline AABB bounds() const { const Vec3f lower(min(lower_x), min(lower_y),min(lower_z));
                               const Vec3f upper(max(upper_x), max(upper_y), max(upper_z));
  			       return AABB(lower, upper); }
  
  vfloat4 lower_x, upper_x, lower_y, upper_y, lower_z, upper_z;
  
};

template<typename I>
struct SetNodeT : public AANode {

  using::Node::children;
  
  using::AANode::lower_x;
  using::AANode::lower_y;
  using::AANode::lower_z;
  using::AANode::upper_x;
  using::AANode::upper_y;
  using::AANode::upper_z;

  
 SetNodeT(const AANode &aanode,
	 const I &setid,
	 const I &fwdID,
	 const I &revID) :
  setID(setid),
    fwdID(fwdID),
    revID(revID)
    { lower_x = aanode.lower_x;
                 lower_y = aanode.lower_y;
		 lower_z = aanode.lower_z;
		 upper_x = aanode.upper_x;
                 upper_y = aanode.upper_y;
		 upper_z = aanode.upper_z;
                 children[0] = aanode.children[0];
                 children[1] = aanode.children[1];
		 children[2] = aanode.children[2];
		 children[3] = aanode.children[3]; }
  
  I setID;
  I fwdID, revID;
  
};

typedef SetNodeT<unsigned> SetNode;

__forceinline std::ostream& operator<<(std::ostream& cout, const AANode &n) {
  return cout <<
         "Lower X's: " << n.lower_x << std::endl <<
         "Upper X's: " << n.upper_x << std::endl <<
         "Lower Y's: " << n.lower_y << std::endl <<
         "Upper Y's: " << n.upper_y << std::endl <<
         "Lower Z's: " << n.lower_z << std::endl <<
         "Upper Z's: " << n.upper_z << std::endl;
}

template<typename I>
__forceinline size_t intersectBox(const AANode &node, const TravRayT<I> &ray, const vfloat4 &tnear, const vfloat4 &tfar, vfloat4 &dist) {
#if defined(__AVX2__)
  const vfloat4 tNearX = msub((vfloat4::load((void*)((const char*)&node.lower_x + ray.nearX))), ray.rdir.x, ray.org_rdir.x);
  const vfloat4 tNearY = msub((vfloat4::load((void*)((const char*)&node.lower_x + ray.nearY))), ray.rdir.y, ray.org_rdir.y);
  const vfloat4 tNearZ = msub((vfloat4::load((void*)((const char*)&node.lower_x + ray.nearZ))), ray.rdir.z, ray.org_rdir.z);
  const vfloat4 tFarX  = msub((vfloat4::load((void*)((const char*)&node.lower_x + ray.farX))) , ray.rdir.x, ray.org_rdir.x);
  const vfloat4 tFarY  = msub((vfloat4::load((void*)((const char*)&node.lower_x + ray.farY))) , ray.rdir.y, ray.org_rdir.y);
  const vfloat4 tFarZ  = msub((vfloat4::load((void*)((const char*)&node.lower_x + ray.farZ))) , ray.rdir.z, ray.org_rdir.z);
#else
  const vfloat4 tNearX = (vfloat4::load((void*)((const char*)&node.lower_x + ray.nearX)) - ray.org.x) * ray.rdir.x;
  const vfloat4 tNearY = (vfloat4::load((void*)((const char*)&node.lower_x + ray.nearY)) - ray.org.y) * ray.rdir.y;
  const vfloat4 tNearZ = (vfloat4::load((void*)((const char*)&node.lower_x + ray.nearZ)) - ray.org.z) * ray.rdir.z;
  const vfloat4 tFarX = (vfloat4::load((void*)((const char*)&node.lower_x + ray.farX)) - ray.org.x) * ray.rdir.x;
  const vfloat4 tFarY = (vfloat4::load((void*)((const char*)&node.lower_x + ray.farY)) - ray.org.y) * ray.rdir.y;
  const vfloat4 tFarZ = (vfloat4::load((void*)((const char*)&node.lower_x + ray.farZ)) - ray.org.z) * ray.rdir.z;
  #endif
  
  const float round_down = 1.0f-2.0f*float(ulp); // FIXME: use per instruction rounding for AVX512
  const float round_up   = 1.0f+2.0f*float(ulp);
  
#if defined(__SSE4_1__)
  const vfloat4 tNear = maxi(tNearX,tNearY,tNearZ,tnear);
  const vfloat4 tFar  = mini(tFarX ,tFarY ,tFarZ ,tfar);
  const vbool4 vmask = round_down*tNear > round_up*tFar;
  //  const vbool4 vmask = asInt(tNear) > asInt(tFar);
  const size_t mask = movemask(vmask) ^ ((1<<4)-1);
#else
  const vfloat4 tNear = max(tNearX, tNearY, tNearZ, tnear);
  const vfloat4 tFar = min(tFarX, tFarY, tFarZ, tfar);
  const vbool4 vmask = (round_down*tNear <= round_up*tFar);
  const size_t mask = movemask(vmask);
#endif
  
  dist = tNear;  
  return mask;
};

template<typename I>
__forceinline size_t nearestOnBox(const AANode &node, const TravRayT<I> &ray, const vfloat4 &tnear, const vfloat4 &tfar, vfloat4 &dist) {

  vfloat4 xval, yval, zval;

  xval = max(node.lower_x, ray.org.x);
  xval = min(node.upper_x, xval);
  
  yval = max(node.lower_y, ray.org.y);
  yval = min(node.upper_y, yval);
  
  zval = max(node.lower_z, ray.org.z);
  zval = min(node.upper_z, zval);

  xval -= ray.org.x;
  yval -= ray.org.y;
  zval -= ray.org.z;

  dist = xval*xval + yval*yval + zval*zval;

  vbool4 mask = tfar > dist;

  return movemask(mask);
  
};

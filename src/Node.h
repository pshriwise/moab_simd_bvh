#pragma once

#include "Vec3.h"
#include "AABB.h"
#include "OBB.h"
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
struct UANode;
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

  __forceinline       UANode* uanode()       { return (UANode*)ptr; }
  __forceinline const UANode* uanode() const { return (const UANode*)ptr; }

  __forceinline       UANode* uasafeNode()        { return isSetLeaf() ? (UANode*)setLeafPtr() : uanode(); }
  __forceinline const UANode* uasafeNode() const  { return isSetLeaf() ? (UANode*)setLeafPtr() : uanode(); }

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
  
  __forceinline AABB bounds() const { const Vec3fa lower(min(lower_x), min(lower_y),min(lower_z));
                                      const Vec3fa upper(max(upper_x), max(upper_y), max(upper_z));
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

  // find the center of the boxes
  const vfloat4 centerX = (node.lower_x+node.upper_x)*0.5;
  const vfloat4 centerY = (node.lower_y+node.upper_y)*0.5;
  const vfloat4 centerZ = (node.lower_z+node.upper_z)*0.5;

  // compute the vector from the ray origin to the box center
  const vfloat4 tminX = node.lower_x - ray.org.x;
  const vfloat4 tminY = node.lower_y - ray.org.y;
  const vfloat4 tminZ = node.lower_z - ray.org.z;
  const vfloat4 tmaxX = ray.org.x - node.upper_x;
  const vfloat4 tmaxY = ray.org.y - node.upper_y;
  const vfloat4 tmaxZ = ray.org.z - node.upper_z;

  vfloat4 tX = max(tminX, tmaxX);
  vfloat4 tY = max(tminY, tmaxY);
  vfloat4 tZ = max(tminZ, tmaxZ);

  dist = max(tX,tY,tZ);

  // we claim to "intersect" all boxes
  return 15;
};

// unaligned node
struct __aligned(16) UANode : public Node {
  
  using::Node::children;

  struct Set {
    __forceinline void operator() (NodeRef node, size_t i, NodeRef child, const OBB& obb) const {
      node.uanode()->setRef(i, child);
      node.uanode()->setBound(i, obb);
    }
  };

  __forceinline void clear() {
    obb.l.vx = Vec3fa(zero);
    obb.l.vy = Vec3fa(zero);
    obb.l.vz = Vec3fa(zero);
  }

  __forceinline void setBounds(const OBB& bounds) {
    for(size_t i = 0; i < N; i++) {
      setBound(i, bounds);
    }
  }
  
  __forceinline void setBound(size_t i, const OBB& bounds) {
    assert(i < N);

    AffineSpace3fa space = bounds.transform;
    space.p -= bounds.bbox.lower;

    space = AffineSpace3fa::scale(1.0f/max(Vec3fa(1E-19f), bounds.bbox.upper - bounds.bbox.lower)) * space;
 
    
    obb.l.vx.x[i] = space.l.vx.x;
    obb.l.vx.y[i] = space.l.vx.y;
    obb.l.vx.z[i] = space.l.vx.z;

    obb.l.vy.x[i] = space.l.vy.x;
    obb.l.vy.y[i] = space.l.vy.y;
    obb.l.vy.z[i] = space.l.vy.z;

    obb.l.vz.x[i] = space.l.vz.x;
    obb.l.vz.y[i] = space.l.vz.y;
    obb.l.vz.z[i] = space.l.vz.z;

    obb.p.x[i] = space.p.x;
    obb.p.y[i] = space.p.y;
    obb.p.z[i] = space.p.z;

    return;
  }

  __forceinline void setRef(size_t i, const NodeRef& ref) {
    assert(i < N);
    children[i] = ref;
  }

  __forceinline Vec3fa extent(size_t i) const {
    assert(i<N);
    const Vec3fa vx(obb.l.vx.x[i],obb.l.vx.y[i],obb.l.vx.z[i]);
    const Vec3fa vy(obb.l.vy.x[i],obb.l.vy.y[i],obb.l.vy.z[i]);
    const Vec3fa vz(obb.l.vz.x[i],obb.l.vz.y[i],obb.l.vz.z[i]);
    return rsqrt(vx*vx + vy*vy + vz*vz);
  }

  __forceinline AABB bounds() const {
    Vec3fa size(0.0);
    Vec3fa llc(inf);
    for(size_t i = 0; i < N; i++) {
      Vec3fa ext = extent(i);
      Vec3fa lower = Vec3fa(obb.p.x[i], obb.p.y[i], obb.p.z[i]);
      lower = lower * ext;
      llc = min(llc, lower);
      size = max(size, ext);
    }
    return AABB(llc, llc + size);
  }
  
 public:
  AffineSpaceV obb;
  
};

template<typename I>
struct OSetNodeT : public UANode {

  using::Node::children;

  using::UANode::obb;
  
 OSetNodeT(const UANode &uanode,
	 const I &setid,
	 const I &fwdID,
	 const I &revID) :
  setID(setid),
    fwdID(fwdID),
    revID(revID)
    { obb = uanode.obb;
      children[0] = uanode.children[0];
      children[1] = uanode.children[1];
      children[2] = uanode.children[2];
      children[3] = uanode.children[3]; }
  
  I setID;
  I fwdID, revID;
  
};



template<typename I>
__forceinline size_t intersectBox(const UANode& node, const TravRayT<I>& ray, const vfloat4& tnear, const vfloat4& tfar, vfloat4& dist) {

  const Vec3vf dir   = xfmVector(node.obb, ray.dir);

  const Vec3vf nrdir = Vec3vf(vfloat4(-1.0f)) * rcp_safe(dir);
  const Vec3vf org   = xfmPoint(node.obb, ray.org);
  const Vec3vf tLowerXYZ = org * nrdir;
  const Vec3vf tUpperXYZ = tLowerXYZ - nrdir;
  
  const vfloat4 tNearX = mini(tLowerXYZ.x,tUpperXYZ.x);
  const vfloat4 tNearY = mini(tLowerXYZ.y,tUpperXYZ.y);
  const vfloat4 tNearZ = mini(tLowerXYZ.z,tUpperXYZ.z);
  const vfloat4 tFarX  = maxi(tLowerXYZ.x,tUpperXYZ.x);
  const vfloat4 tFarY  = maxi(tLowerXYZ.y,tUpperXYZ.y);
  const vfloat4 tFarZ  = maxi(tLowerXYZ.z,tUpperXYZ.z);
  const vfloat4 tNear  = max(tnear, tNearX,tNearY,tNearZ);
  const vfloat4 tFar   = min(tfar,  tFarX ,tFarY ,tFarZ );
  const vbool4 vmask = tNear <= tFar;
  dist = tNear;
  return movemask(vmask);  
}


template<typename I>
__forceinline size_t nearestOnBox(const UANode& node, const TravRayT<I>& ray, const vfloat4& tnear, const vfloat4& tfar, vfloat4& dist) {
  
  const Vec3vf dir   = xfmVector(node.obb, ray.dir);

  const Vec3vf nrdir = Vec3vf(vfloat4(1.0f)) * rcp_safe(dir);
  const Vec3vf org   = xfmPoint(node.obb, ray.org);
  const Vec3vf tLowerXYZ = org * nrdir;
  const Vec3vf tUpperXYZ = tLowerXYZ - nrdir;

  const vfloat4 tNearX = mini(tLowerXYZ.x,tUpperXYZ.x);
  const vfloat4 tNearY = mini(tLowerXYZ.y,tUpperXYZ.y);
  const vfloat4 tNearZ = mini(tLowerXYZ.z,tUpperXYZ.z);
  const vfloat4 tFarX  = maxi(tLowerXYZ.x,tUpperXYZ.x);
  const vfloat4 tFarY  = maxi(tLowerXYZ.y,tUpperXYZ.y);
  const vfloat4 tFarZ  = maxi(tLowerXYZ.z,tUpperXYZ.z);
  const vfloat4 tNear  = max(tnear, tNearX,tNearY,tNearZ);
  const vfloat4 tFar   = min(tfar,  tFarX ,tFarY ,tFarZ );
  const vbool4 vmask = tNear <= tFar;
  dist = max(tNear,tFar);
  
  return 15;
}

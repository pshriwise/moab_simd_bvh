#pragma once

#include "Vec3.h"
#include "AABB.h"
#include "constants.h"
#include "vfloat.h"
#include "Primitive.h"



static const size_t emptyNode = 8;
static const size_t tyLeaf = 8;
static const size_t setLeaf = 3;

static const size_t items_mask = 15;
static const size_t align_mask = 15;

 // forward declarations
struct SetNode;
struct AANode;
struct Node;
 
struct NodeRef {

  inline NodeRef () { ptr = emptyNode;}

  inline NodeRef(size_t ptr) : ptr(ptr) {}

  inline operator size_t() const { return ptr; }

  inline size_t pointer () const { return ptr; }
  
  inline size_t isLeaf() const { return ptr & tyLeaf; }

  inline size_t isSetLeaf() const { return !(isLeaf()) && (ptr & setLeaf); }

  inline bool isEmpty() const { return ptr == emptyNode; }

  inline       AANode* node()       { return (AANode*)ptr; }
  inline const AANode* node() const { return (const AANode*)ptr; }

  inline       SetNode* snode()       { return (SetNode*)ptr; }
  inline const SetNode* snode() const { return (const SetNode*)ptr; }
  
  inline       Node* bnode()       { return (Node*)ptr; }
  inline const Node* bnode() const { return (const Node*)ptr; }

  inline void* leaf(size_t& num) const {
    assert(isLeaf());
    num = 1 + (ptr & (items_mask))-tyLeaf;
    return (void*) (ptr & ~(size_t)align_mask);
  }
  
  /* inline char* leaf(size_t& num) const { */
  /*   assert(isLeaf()); */
  /*   num = (ptr & (size_t)items_mask)-tyLeaf; */
  /*   return (char*)(ptr & ~(size_t)align_mask); */
  /* } */

  
private:
  size_t ptr;
		   

};


inline const bool operator ==( const NodeRef& a, const NodeRef& b ) { return a.pointer() == b.pointer(); }



struct Node {
  inline Node () {}
  
  inline void clear() { for(size_t i=0; i < N; i++) children[i] = emptyNode; }

  inline bool verify() {
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
  
  inline NodeRef& child(size_t i) { assert(i<N); return children[i]; }
  inline const NodeRef& child(size_t i) const { assert(i<N); return children[i]; }

  NodeRef children[N];
  
};



struct AANode : public Node
{

  using::Node::children;


  // inline AANode& child(size_t i) { assert(i<N); return children[i]; }
  //  inline const AANode& child(size_t i) const { assert(i<N); return children[i]; }

  // empty constructor
  inline AANode() {}

  inline AANode( const vfloat4& low_x, const vfloat4& up_x,
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

  inline void set( const vfloat4& low_x, const vfloat4& up_x,
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


struct SetNode : public AANode {

  using::AANode::lower_x;
  using::AANode::lower_y;
  using::AANode::lower_z;
  using::AANode::upper_x;
  using::AANode::upper_y;
  using::AANode::upper_z;

  using::Node::children;
  
 SetNode(const AANode &aanode, const unsigned &setid) :
  setID(setid) { lower_x = aanode.lower_x;
                 lower_y = aanode.lower_y;
		 lower_z = aanode.lower_z;
		 upper_x = aanode.upper_x;
                 upper_y = aanode.upper_y;
		 upper_z = aanode.upper_z;
                 children[0] = aanode.children[0];
                 children[1] = aanode.children[1];
		 children[2] = aanode.children[2];
		 children[3] = aanode.children[3]; }
  
  unsigned setID;
  unsigned fwdID, bwdID;
  
};

inline std::ostream& operator<<(std::ostream& cout, const AANode &n) {
  return cout <<
         "Lower X's: " << n.lower_x << std::endl <<
         "Upper X's: " << n.upper_x << std::endl <<
         "Lower Y's: " << n.lower_y << std::endl <<
         "Upper Y's: " << n.upper_y << std::endl <<
         "Lower Z's: " << n.lower_z << std::endl <<
         "Upper Z's: " << n.upper_z << std::endl;
}

  
inline size_t intersectBox(const AANode &node, const TravRay &ray, const vfloat4 &tnear, const vfloat4 &tfar, vfloat4 &dist) {
  const vfloat4 tNearX = (vfloat4::load((void*)((const char*)&node.lower_x + ray.nearX))- ray.org.x) * ray.rdir.x;
  const vfloat4 tNearY = (vfloat4::load((void*)((const char*)&node.lower_x + ray.nearY)) - ray.org.y) * ray.rdir.y;
  const vfloat4 tNearZ = (vfloat4::load((void*)((const char*)&node.lower_x + ray.nearZ)) - ray.org.z) * ray.rdir.z;
  const vfloat4 tFarX = (vfloat4::load((void*)((const char*)&node.lower_x + ray.farX)) - ray.org.x) * ray.rdir.x;
  const vfloat4 tFarY = (vfloat4::load((void*)((const char*)&node.lower_x + ray.farY)) - ray.org.y) * ray.rdir.y;
  const vfloat4 tFarZ = (vfloat4::load((void*)((const char*)&node.lower_x + ray.farZ)) - ray.org.z) * ray.rdir.z;
  
  const float round_down = 1.0f-2.0f*float(ulp); // FIXME: use per instruction rounding for AVX512
  const float round_up   = 1.0f+2.0f*float(ulp);

  const vfloat4 tNear = max(tNearX, tNearY, tNearZ, tnear);
  const vfloat4 tFar = min(tFarX, tFarY, tFarZ, tfar);

  const vbool4 vmask = (round_down*tNear <= round_up*tFar);
  dist = tNear;
  const size_t mask = movemask(vmask);
  return mask;
};


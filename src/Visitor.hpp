
#include "Node.h"

template<typename V, typename P, typename I>
class BVHOperator {

public:

  typedef RayT<V,P,I> Ray;
  
  virtual bool visit(NodeRef& current_node, TravRay vray, const vfloat4& tnear, const vfloat4& tfar, vfloat4& tNear, size_t& mask) = 0;

  virtual void setLeaf(NodeRef current_node) = 0;
  
  virtual void leaf(NodeRef current_node, NodeRef previous_node, Ray ray) = 0;

};



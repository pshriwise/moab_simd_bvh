
#pragma once

#include "Node.h"

template<typename V, typename P, typename I>
class BVHOperator {

public:

  typedef RayT<V,P,I> Ray;
  
  virtual bool visit(NodeRef current_node, TravRay& vray, const vfloat4& tnear, const vfloat4& tfar, vfloat4& tNear, size_t& mask) = 0;

  virtual void setLeaf(NodeRef current_node, const NodeRef& previous_node) = 0;
  
  virtual void leaf(NodeRef current_node, const NodeRef& previous_node, const NodeRef& last_set_leaf, Ray& ray) = 0;

};



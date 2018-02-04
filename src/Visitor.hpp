
#include "Node.h"

class BVHOperator {

public:
  
  virtual bool visit(NodeRef& current_node, size_t& mask, vfloat4& tnear) = 0;

  virtual void setLeaf(NodeRef current_node) = 0;
  
  virtual void leaf(NodeRef current_node) = 0;

};




#include "Node.h"

class BVHOperator {
  
  virtual void visit(NodeRef current_node, size_t& mask);

  virtual void setLeaf(NodeRef current_node);
  
  virtual void leaf(NodeRef current_node);

};




#include "TraversalVisitor.hpp"
#include "moab/Core.hpp"

class BaseVisitor : public BVHOperator<Vec3da,double,moab::EntityHandle> {

public:
  BaseVisitor(moab::Interface* mbi) {
    orig_mbi = mbi;
  }
  
  // MOAB instance used to load file and build the tree
  moab::Interface* orig_mbi;

  ~BaseVisitor() {}
  
public:
  
  moab::Interface* original_mbi() { return orig_mbi; }
  
  int find_child_number(NodeRef current_node, NodeRef previous_node) {
    
    const Node* node = previous_node.bnode();
    
    // find what "number" this child is
    size_t child_number = -1;
    for(size_t i = 0; i < N; i++){
      if ( node->child(i) == current_node ) {
	child_number = i;
	break;
      }
    }
    assert( child_number >= 0 );

    return child_number;
  }  

};

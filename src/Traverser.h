#pragma once


#include "Node.h"
#include "Stack.h"
#include "Ray.h"
#include "ops.h"

//#define VERBOSE_MODE

class BVHTraverser {
 public:

  static inline void traverseClosest(NodeRef& current_node, 
			             size_t mask,
				     const vfloat4& tNear,
				     StackItemT<NodeRef>*& stackPtr,
				     StackItemT<NodeRef>* stackEnd)
  {
    assert(mask != 0);
#ifdef VERBOSE_MODE
    std::cout << "One Hit" << std::endl;
#endif
    const Node* node = current_node.bnode();

    /* check for single child hit, add child to stack */
    size_t r = __bscf(mask);
    current_node = node->child(r);
    // prefetch here
    if(mask == 0) {
      // check for emptyNode here
      return;
    }

    //if there is another child node hit, push far child into the stack and continue with the closer child
    NodeRef c0 = current_node;
    const unsigned int d0 = ((unsigned int*)&tNear)[r];
    r = __bscf(mask);
    NodeRef c1 = node->child(r);
    // prefectch c1 here
    const unsigned int d1 = ((unsigned int*)&tNear)[r];
#ifdef VERBOSE_MODE
    std::cout << "Two Hits" << std::endl;
    std::cout << d0 << std::endl;
    std::cout << d1 << std::endl;;
#endif
    if(mask == 0) {
      if ( d0 < d1 ) { stackPtr-> ptr = c1; stackPtr-> dist = d1; stackPtr++; current_node = c0; return; }
      else           { stackPtr-> ptr = c0; stackPtr-> dist = d0; stackPtr++; current_node = c1; return; }
    }

    // here starts 3 or 4 hits. put nodes on stack. They will be sorted there from now on
    stackPtr->ptr = c0; stackPtr->dist = d0; stackPtr++;
    stackPtr->ptr = c1; stackPtr->dist = d1; stackPtr++;

    //locate next node hit
    r = __bscf(mask);
    NodeRef c = node->child(r);
    //prefecth here
    unsigned int d = ((unsigned int*)&tNear)[r];
#ifdef VERBOSE_MODE
    std::cout << "Three Hits" << std::endl;
    std::cout << d << std::endl;
#endif 
    stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;

    // if this is the last hit
    if(mask == 0) {
      sort(stackPtr[-1], stackPtr[-2], stackPtr[-3]);
      current_node = (NodeRef) stackPtr[-1].ptr; stackPtr--;
      return;
  }

    // all four children are hit, push all nodes onto stack, sort, return closest child
    r = __bscf(mask);
    c = node->child(r);
    d = ((unsigned int*)&tNear)[r];
#ifdef VERBOSE_MODE
    std::cout << "Four Hits" << std::endl;
    std::cout << d << std::endl;
#endif 
    stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
    sort(stackPtr[-1], stackPtr[-2], stackPtr[-3], stackPtr[-4]);
    current_node = (NodeRef) stackPtr[-1].ptr; stackPtr--;
  }
    
  
};

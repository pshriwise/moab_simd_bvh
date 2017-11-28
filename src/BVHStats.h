
#pragma once

#include <vector>

#include "Ray.h"
#include "Node.h"
#include "Stack.h"

struct BVHStatTracker {
  
  std::vector<int> s;

  int num_empty;
  int num_leaves;
  
  static const size_t stackSize = 1+(N-1)*BVH_MAX_DEPTH;

  inline BVHStatTracker() : num_empty(0), num_leaves(0) { s.push_back(1); }

  inline void count_hits(size_t& mask, int& hits) {
    assert(mask <= 15 && mask >= 0);
    
    if(mask == 0) return;
    size_t r = __bscf(mask);
    hits++;
    count_hits(mask, hits);
    return;
  }
  
  inline void down(size_t mask) {
    assert(mask >= 0);

    s.back()--;

    if(mask == 0 ) return;
    
    int nodes_hit = 0;
    count_hits(mask, nodes_hit);
    s.push_back(nodes_hit);
    
    return;
  }
  
  inline void up() {

    if(s.back() == 0) {
      s.pop_back();
      up();
    }
    else{
      return;
    }
  }

  inline int depth() { return s.size()-1; }


  inline void gatherStats(NodeRef root) {

        /* initialiez stack state */
    StackItemT<NodeRef> stack[stackSize];
    StackItemT<NodeRef>* stackPtr = stack+1;
    StackItemT<NodeRef>* stackEnd = stack+stackSize;
    stack[0].ptr = root;
    stack[0].dist = neg_inf;

    TraversalTracker t;

    BVHTraverser nodeTraverser;
    new (&nodeTraverser) BVHTraverser();

    while (true) pop:
      {
	if(stackPtr == stack) break;
	stackPtr--;
	NodeRef cur = NodeRef(stackPtr->ptr);

	while (true)
	  {
	    size_t mask = 8; //always visit all child nodes

	    if( !cur.isEmpty() ) num_empty++;

	    if( !cur.isLeaf() ) {
	      num_leaves++;
	      break;
	    }

	    nodeTraverser.traverse(cur, mask, stackPtr, stackEnd);

	  }

	size_t numPrims;
	void* prims = cur.leaf(numPrims);

      }
  }
  
};

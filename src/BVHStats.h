
#pragma once

#include <vector>

#include "Ray.h"
#include "Node.h"
#include "Stack.h"

struct BVHStatTracker {
  
  std::vector<int> s;

  
  int num_empty;
  int num_non_empty;
  int num_leaves;
  int depth;
  int max_leaf;
  int min_leaf;
  int total_entities;

  int min_leaf_depth;
  int max_leaf_depth;
  int leaf_depth_agg;
  
  static const size_t stackSize = 1+(N-1)*BVH_MAX_DEPTH;

  inline BVHStatTracker() : num_empty(0),
			    num_non_empty(0),
			    num_leaves(0),
			    depth(0),
			    min_leaf(inf),
			    max_leaf(0),
			    total_entities(0),
			    min_leaf_depth(inf),
			    max_leaf_depth(0),
			    leaf_depth_agg(0)
  { s.push_back(1); }

  inline void count_hits(size_t mask, int& hits) {
    assert(mask <= 15 && mask >= 0);
    
    if(mask == 0) return;
    size_t r = __bscf(mask);
    hits++;
    count_hits(mask, hits);
    return;
  }
  
  inline void down(size_t mask) {
    assert(mask >= 0);

    if( mask == 0 ) return;
    
    int nodes_hit = 0;
    count_hits(mask, nodes_hit);
    s.push_back(nodes_hit-1);
    
    return;
  }
  
  inline void up() {

    if(s.back() == 0) {
      s.pop_back();
      up();
    }
    else{
      s.back()--;
      return;
    }
  }

  inline int current_depth() { return s.size()-1; }


  inline void gatherStats(NodeRef root) {

        /* initialiez stack state */
    StackItemT<NodeRef> stack[stackSize];
    StackItemT<NodeRef>* stackPtr = stack+1;
    StackItemT<NodeRef>* stackEnd = stack+stackSize;
    stack[0].ptr = root;
    stack[0].dist = neg_inf;

    BVHTraverser nodeTraverser;
    new (&nodeTraverser) BVHTraverser();

    while (true) pop:
      {
	if(stackPtr == stack) break;
	stackPtr--;
	NodeRef cur = NodeRef(stackPtr->ptr);
	up();
	
	while (true)
	  {
	    size_t mask = 15; //always visit all child nodes

	    if( cur.isEmpty() ) num_empty++;
	    else num_non_empty++;
	    down(mask);
	    
	    if( cur.isLeaf() ) {
	      depth = current_depth() > depth ? current_depth() : depth;
	      num_leaves++;
	      break;
	    }
	    
	    if (mask == 0) goto pop;
	    
	    nodeTraverser.traverse(cur, mask, stackPtr, stackEnd);

	  }

	size_t numPrims;
	void* prims = cur.leaf(numPrims);
	max_leaf = numPrims > max_leaf ? numPrims : max_leaf;
	min_leaf = numPrims > 0 && numPrims < min_leaf ? numPrims : min_leaf;

	max_leaf_depth = current_depth() > max_leaf_depth ? current_depth() : max_leaf_depth;
	min_leaf_depth = current_depth() < min_leaf_depth ? current_depth() : min_leaf_depth;
	
	total_entities += numPrims;
	leaf_depth_agg += current_depth();
      }

    std::cout << "Tree depth: " << depth << std::endl;
    std::cout << "Total number of entiites in the tree: " << total_entities << std::endl;
    std::cout << "Number of leaves in the tree: " << num_leaves << std::endl;
    std::cout << "Number of non-empty nodes in the tree: " << num_non_empty << std::endl;    
    std::cout << "Number of empty nodes in the tree: " << num_empty << std::endl;
    std::cout << "Minimum entities in non-empty leaf: " << min_leaf << std::endl;
    std::cout << "Maximum entities in leaf: " << max_leaf << std::endl;
    std::cout << "Average entities in leaf: " << (double)total_entities/(double)num_leaves << std::endl;
    std::cout << "Maximum leaf depth: " << max_leaf_depth << std::endl;
    std::cout << "Minimum leaf depth: " << min_leaf_depth << std::endl;
    std::cout << "Average leaf depth: " << (double)leaf_depth_agg/(double)num_leaves << std::endl;
  }
  
};



#include "Intersector.h"
#include "Traverser.h"
#include "Stack.h"

class BVHTraverser;

void BVHIntersector::intersectRay(NodeRef root, Ray& ray) {

  /* initialiez stack state */
  StackItemT<NodeRef> stack[stackSize];
  StackItemT<NodeRef>* stackPtr = stack+1;
  StackItemT<NodeRef>* stackEnd = stack+stackSize;
  stack[0].ptr = root;
  stack[0].dist = neg_inf;

  /* verify correct inputs */
  assert(ray.valid());
  assert(ray.tnear >= 0.0f);

  TravRay vray = TravRay(ray.org, ray.dir);
  vfloat4 ray_near = max(ray.tnear, 0.0f);
  vfloat4 ray_far = max(ray.tfar, 0.0f);

  BVHTraverser nodeTraverser;
  
  while (true) pop:
    {
      if(stackPtr == stack) break;
      stackPtr--;
      NodeRef cur = NodeRef(stackPtr->ptr);

      // if the ray doesn't reach this node, move to next
      if(*(float*)&stackPtr->dist > ray.tfar) { continue; }
      
      
      while (true)
	{
	  size_t mask; vfloat4 tNear;
	  bool nodeIntersected = intersect(cur, vray, ray_near, ray_far, tNear, mask);
	  // if no intersection, move on to the next node in the stack
	  if (!nodeIntersected) { break; }
	  
	  // if no children were hit, pop next node
	  if (mask == 0) { goto pop; }

	  nodeTraverser.traverseClosest(cur, mask, tNear, stackPtr, stackEnd);
	}

      // leaf (set distance to nearest/farthest box intersection for now)
      
    }
  

}



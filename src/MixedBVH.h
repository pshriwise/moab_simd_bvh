
#pragma once

#include <set>
#include <vector>
#include <bitset>

//#include "Builder.h"
#include "BuildState.h"
#include "Intersector.h"
#include "TriangleRef.h"
#include "FilterFunc.h"
#include "MOABDirectAccessManager.h"
#include "BVHStats.h"
#include "BVHSettings.h"
#include "PrimitiveReference.h"

#include "BVHRefiner.h"

#define MAX_LEAF_SIZE 8

template <typename V, typename T, typename I>
class MixedBVH {

  typedef BVHSettingsT<PrimRef, AABB> BVHSettings;
  typedef BVHSettingsT<NodeRef*, AABB> BVHJoinTreeSettings;

  typedef SetNodeT<I> SetNode;
  
  typedef TempNodeT<PrimRef, AABB> TempPrimNode;
  typedef TempNodeT<NodeRef*, AABB> TempSetNode;

  typedef BuildStateT<PrimRef> BuildState;

  typedef MBTriangleRefT<V, T, I> P;
  
  typedef TravRayT<I> TravRay;
  typedef RayT<V,T,I> Ray;

  typedef BVHRefinerT<V, T, I> BVHRefiner;
  
 public:
  typedef FilterT<V,double,I> Filter;
  typename Filter::FilterFunc filter;

 private:
  static void no_filter(Ray &ray, void* mesh_ptr) { return; };

 public:
  inline MixedBVH(MOABDirectAccessManager *mdam) : MDAM(mdam), maxLeafSize(8), depth(0), maxDepth(BVH_MAX_DEPTH), num_stored(0), filter(&no_filter)
    {
      std::vector<P> storage_vec(MDAM->num_elements);
      leaf_sequence_storage = storage_vec;
      //      leaf_sequence_storage.resize(MDAM->num_elements);
      stack = (StackItemT<NodeRef>*)malloc(sizeof(StackItemT<NodeRef>)*stackSize);
      
      RefinerTool = new BVHRefiner(mdam);
    }

  inline ~MixedBVH() { free(stack); }
  
 private:
  
  size_t maxLeafSize;
  size_t depth;
  size_t maxDepth;

  int num_stored;

  std::vector<P> leaf_sequence_storage;

  MOABDirectAccessManager* MDAM;
  BVHRefiner *RefinerTool;
  
  static const size_t stackSize = 1+N*BVH_MAX_DEPTH;
  StackItemT<NodeRef>* stack;
  
 public:

  inline void set_filter(typename Filter::FilterFunc ff) { filter = ff; }

  inline void unset_filter() { filter = no_filter; }

  
  /// leaf encoding ///
  // this function takes in a pointer to
  // the location at which a set of primitive references are
  // stored as part of the tree structure. Because primitives
  // are required to take up 16 bytes, the number of
  // primitives is stored in the THREE LEAST SIGNIFICANT bits of
  // the ptr attribute in the returned node reference.
  // The node is also marked as a leaf using the tyLeaf value
  // which should not interfere with the other encoded bytes,
  // being the FOURTH leas significant bit.
  NodeRef* encodeLeaf(void *prim_arr, size_t num) {
    assert(num < MAX_LEAF_SIZE);
    return new NodeRef((size_t)prim_arr, (tyLeaf + num));
  }
  
  inline void makeSetNode(NodeRef* node, I setID, I fwd = 0, I rev = 0) {

    //make sure this isn't already a set node
    assert(!node->isSetLeaf());
    
    // replace this normal root node with a set node
    /* AANode *aanode = new AANode(); */
    /* AABB node_box = box_from_node(node); */
    /* aanode->setBounds(node_box); */
    SetNode* snode = new SetNode(node->pointer(), setID, fwd, rev);
        
    /* if( node->isLeaf() ) { */
    /*   snode->setRef(0,*node); */
    /*   snode->setRef(1,NodeRef()); */
    /*   snode->setRef(2,NodeRef()); */
    /*   snode->setRef(3,NodeRef()); */
    /* } */
    /* else { */
    /*   snode->setRef(0,node->bnode()->child(0)); */
    /*   snode->setRef(1,node->bnode()->child(1)); */
    /*   snode->setRef(2,node->bnode()->child(2)); */
    /*   snode->setRef(3,node->bnode()->child(3)); */
    /*   delete node->anyNode(); */
    /* } */
    
    node->setPtr((size_t)snode | setLeafAlign);

    return;
  }
  
  inline void* createLeaf(P* primitives, size_t numPrimitives) {
    return (void*) encodeLeaf((void*)primitives, numPrimitives - 1);
  }

  inline void split_sets(NodeRef* current_node, size_t split_dim, NodeRef** nodesPtr, size_t numNodes, TempSetNode child_nodes[N]) {

    // get the current node's bounds
    AABB box = ((AANode*)current_node->anyNode())->bounds();

    AABB boxes[N];

    boxes[0]= box.splitBox(split_dim, 0.00, 0.25);
    boxes[1]= box.splitBox(split_dim, 0.25, 0.50);
    boxes[2]= box.splitBox(split_dim, 0.50, 0.75);
    boxes[3]= box.splitBox(split_dim, 0.70, 1.00);
    
    child_nodes[0].clear();
    child_nodes[1].clear();
    child_nodes[2].clear();
    child_nodes[3].clear();
	

    for(size_t i = 0; i < numNodes; i ++) {
      AANode* aanode = (AANode*)nodesPtr[i]->anyNode();
      bool placed = false;
      
      for(size_t j = 0; j < N; j++){
	AABB node_box = box_from_node(nodesPtr[i]);

	if( inside( boxes[j], node_box.center() ) ){
	  placed = true;
	  child_nodes[j].push_back(nodesPtr[i]);
	  break;
	}
      }
      assert(placed);
    }

    return;
  }

  inline AABB box_from_nodes(NodeRef** nodesPtr, size_t numNodes) {
    AABB nodes_box;
    nodes_box.clear();
    for(size_t i = 0; i < numNodes; i++){
      nodes_box.extend(box_from_node(nodesPtr[i]));
    }
    return nodes_box;
  }
  
  inline AABB box_from_node(NodeRef* node) {

    
    NodeRef node_ref = *node;

    // if this is a SetNode, redirect the node_ref appropriately
    if(node_ref.isSetLeaf()){
      SetNode* snode = (SetNode*)node_ref.snode();
      node_ref = snode->ref();
    }

    AABB node_box;
    if(!node_ref.isLeaf()){
      AANode* temp_node = (AANode*)node_ref.anyNode();
      node_box = temp_node->bounds();
    }
    else {
      // get the primitives
      size_t numPrims;
      P* primIDs = (P*)node_ref.leaf(numPrims);
      for (size_t j = 0; j < numPrims; j++){
	P t = primIDs[j];
	Vec3fa lower, upper;
	t.get_bounds(lower, upper, MDAM);
	AABB box = AABB(lower, upper);
	box.bump(BOX_BUMP);
	node_box.update(box);
      }
    }
    return node_box;
  }
  
  inline void split_sets(NodeRef* current_node, NodeRef** nodesPtr, size_t numNodes, TempSetNode child_nodes[N], BVHJoinTreeSettings* settings) {

    int best_dim;
    float best_cost = 1.0;
    float cost;

    AABB node_box = ((AANode*)current_node->anyNode())->bounds();
    // attempt split along each axis
    for(size_t i = 0; i < 3; i++) {
      split_sets(current_node, i, nodesPtr, numNodes, child_nodes);
      cost = settings->evaluate_cost(child_nodes, node_box, numNodes);      
      if ( cost <= best_cost ) {
	best_cost = cost;
	best_dim = i;
      }
    }

    split_sets(current_node, best_dim, nodesPtr, numNodes, child_nodes);

    return;
    
  }

  inline NodeRef* join_trees( std::vector<NodeRef*> nodes, BVHJoinTreeSettings* settings = NULL) {
    if(!settings) settings = new BVHJoinTreeSettings();
    settings->set_heuristic(ENTITY_RATIO_HEURISTIC);
    
    return join_trees( &(nodes[0]), (size_t)nodes.size(), settings );
  }
  
  inline NodeRef* join_trees(NodeRef** nodesPtr, size_t numNodes, BVHJoinTreeSettings* settings) {
    
    if (numNodes == 1) {
      return nodesPtr[0];
    }

    if (numNodes == 0) {
      return new NodeRef();
    }

    AANode* aanode = new AANode();
    AABB box = box_from_nodes(nodesPtr, numNodes);
    aanode->setBounds(box);
    
    NodeRef* this_node = new NodeRef((size_t)aanode, ALIGNED_NODE);
    
    TempSetNode child_nodes[N];
    split_sets(this_node, nodesPtr, numNodes, child_nodes, settings);

    // need arbitrary split check here
    for(size_t i = 0; i < N; i++) {
      if (child_nodes[i].size() == numNodes) {
	child_nodes[0].clear();
	child_nodes[1].clear();
	child_nodes[2].clear();
	child_nodes[3].clear();
	//arb split
	int i = 0;
	NodeRef** it = nodesPtr;
	NodeRef** end = nodesPtr+(numNodes
				  );
	while(it != end) {
	  child_nodes[i].push_back(*it);
	  i++;
	  it++;
	  if (i == N) { i = 0; }
	}
	break;
      }
    }

    for(size_t i = 0; i < N; i++) {
      size_t num_child_prims = child_nodes[i].size();
      AABB box;
      if(num_child_prims == 0) {
	box = AABB(0.0f);
      }
      else {
	box = box_from_nodes(&child_nodes[i].prims.front(), num_child_prims);
      }
      assert(box.isValid());
      aanode->setBound(i, box);
    }
    
    for(size_t i = 0; i < N; i++) {
      NodeRef* child_node = join_trees(child_nodes[i].prims, settings);
      aanode->setRef(i, *child_node);
    }

    return this_node;
  }

  inline NodeRef* Build(I* id, size_t numPrimitives, BVHSettings* settings = NULL) {
    // create BuildState of PrimitiveReferences

    // if no primitives are passed, return a node pointing to emptt leaves
    if(numPrimitives == 0) {
      AANode *aanode = new AANode();
      aanode->setRef(0,NodeRef());
      aanode->setRef(1,NodeRef());
      aanode->setRef(2,NodeRef());
      aanode->setRef(3,NodeRef());
      NodeRef* node = new NodeRef((size_t)aanode, ALIGNED_NODE);
      return node;
    }
    
    BuildState bs(0);
    for( size_t i = 0; i < numPrimitives; i++ ) {
      I handle = *(id+i);
      int index = handle - MDAM->first_element;
      
      P triref = P((I*)MDAM->conn + (index*MDAM->element_stride), handle);
      
      Vec3fa lower, upper;
      
      triref.get_bounds(lower, upper, MDAM);
	
      PrimRef p(lower, upper, (void*)triref.eh, index);

      bs.prims.push_back(p);
    }

    // if the settings pointer is null, create a settings struct
    if(!settings) settings = new BVHSettings();
    
    NodeRef *root = Build(bs, settings);
    
    delete settings;
    
    return root;
  }

  inline NodeRef* Build(BuildState& current, BVHSettings *settings) {

    const PrimRef* primitives = current.ptr();
    size_t numPrimitives = current.size();
    
    // if the end conditions for the tree are met, then create a leaf
    if(numPrimitives <= maxLeafSize || current.depth > maxDepth) {
#ifdef VERBOSE_MODE
      std::cout << "Sending " << current.size() << std::endl;
#endif
      return createLargeLeaf(current);
    }
    
    // created a new node and set the bounds
    AANode* aanode = new AANode();
    AABB box = AABB((float)inf, (float)neg_inf);
    for(size_t i = 0; i < numPrimitives; i++) {
      box.update(primitives[i].lower.x, primitives[i].lower.y, primitives[i].lower.z);
      box.update(primitives[i].upper.x, primitives[i].upper.y, primitives[i].upper.z);
    }
    box.bump(BOX_BUMP);
    // increment depth and recur here
    aanode->setBounds(box);

    NodeRef* this_node = RefinerTool->refine(current);
    if(this_node) { return this_node; }
      

    this_node = new NodeRef((size_t)aanode, ALIGNED_NODE);
    TempPrimNode tempNodes[4];
    splitNode(this_node, primitives, numPrimitives, tempNodes, settings);

#ifdef VERBOSE_MODE
    std::cout << "New Node with Bounds: " << std::endl << *aanode << std::endl;
    std::cout << "At depth: " << depth << std::endl;
#endif

    for(size_t i = 0; i < N ; i++){
      BuildState br(current.depth+1, tempNodes[i].prims);
      NodeRef* child_node = Build(br, settings);
      // link the child node
      aanode->setRef(i, *child_node);
      delete child_node;
    }

    depth = current.depth > depth ? current.depth : depth;

    tempNodes[0].clear(); tempNodes[0].prims.shrink_to_fit();
    tempNodes[1].clear(); tempNodes[1].prims.shrink_to_fit();
    tempNodes[2].clear(); tempNodes[2].prims.shrink_to_fit();
    tempNodes[3].clear(); tempNodes[3].prims.shrink_to_fit();

    
    return this_node;
  } // end build


  void splitNode(NodeRef* node, const PrimRef* primitives, const size_t numPrimitives, TempPrimNode tempNodes[N], BVHSettings *settings) {

    // split node along each axis
    float max_cost = 2.0;
    float min_cost = 0.0;
  
    float best_cost = max_cost;
    int best_dim = -1;
    float cost;

    AANode* this_node = (AANode*)node->anyNode();

    AABB node_box = this_node->bounds();

    size_t np = numPrimitives;
    // split along each axis and get lowest cost
    for(size_t i = 0; i < 3; i++) {
      splitNode(node, i, primitives, numPrimitives, tempNodes);
      //compute the SAH cost of this node split
      cost = settings->evaluate_cost(tempNodes, node_box, np);
      assert(cost >= min_cost && cost <= max_cost);
      // update cost
      if (cost < best_cost) {
	best_cost = cost;
	best_dim = i;
      }
    }

    assert(best_dim != -1);

    splitNode(node, best_dim, primitives, numPrimitives, tempNodes);

    tempNodes[0].box.bump(BOX_BUMP);
    tempNodes[1].box.bump(BOX_BUMP);
    tempNodes[2].box.bump(BOX_BUMP);
    tempNodes[3].box.bump(BOX_BUMP);

    for(size_t i = 0; i < N; i++) {
      this_node->setBound(i, tempNodes[i].box);
    }
    
    return;
  
  }

  void splitNode(NodeRef* node, size_t split_axis, const PrimRef* primitives, const size_t numPrimitives, TempPrimNode tn[N]) {

    assert(split_axis >= 0 && split_axis <= 2);

    //get the bounds of the node
    AABB box((float)inf,(float)neg_inf);
    for(size_t i=0; i<numPrimitives; i++) {
      box.update(primitives[i].lower.x,primitives[i].lower.y,primitives[i].lower.z);
      box.update(primitives[i].upper.x,primitives[i].upper.y,primitives[i].upper.z);
    }

    AABB boxes[N];

    boxes[0]= box.splitBox(split_axis, 0.00, 0.25);
    boxes[1]= box.splitBox(split_axis, 0.25, 0.50);
    boxes[2]= box.splitBox(split_axis, 0.50, 0.75);
    boxes[3]= box.splitBox(split_axis, 0.70, 1.00);

    tn[0].clear(); tn[0].prims.shrink_to_fit();
    tn[1].clear(); tn[1].prims.shrink_to_fit();
    tn[2].clear(); tn[2].prims.shrink_to_fit();
    tn[3].clear(); tn[3].prims.shrink_to_fit();

    // sort primitives into boxes by their centroid
    for(size_t i = 0; i < numPrimitives; i++) {
      const PrimRef* p = &(primitives[i]);
      bool placed = false;
      for(size_t j = 0; j < 4 ; j++) {
	// if the centroid is in the box, place it there
	if( inside(boxes[j], p->center()) ){
	  placed = true;
	  tn[j].prims.push_back(*p);
	  tn[j].box.update(p->lower.x, p->lower.y, p->lower.z);
	  tn[j].box.update(p->upper.x, p->upper.y, p->upper.z);
	  break;
	}
      }
      assert(placed);
    }

    tn[0].box.bump(BOX_BUMP);
    tn[1].box.bump(BOX_BUMP);
    tn[2].box.bump(BOX_BUMP);
    tn[3].box.bump(BOX_BUMP);
    
    return;
  }


  NodeRef* createLargeLeaf(BuildState& current) {
    
    /* if(current.depth > maxDepth) { */
    /*   std::cerr << "Maximum depth reached" << std::endl; */
    /*   std::cerr << "Current depth: " << current.depth << std::endl; */
    /*   std::cerr << "Maximum allowed depth: " << maxDepth << std::endl; */
    /*   std::cerr << "Number of primitives remaining: " << current.size() << std::endl; */
    /*   assert(false); */
    /* } */

    if (current.size() <= maxLeafSize) {
      depth = current.depth > depth ? current.depth : depth;

      if(current.size() == 0 ) return new NodeRef();

      P* position = &(*(leaf_sequence_storage.begin()+num_stored));

      for( size_t i = 0; i < current.size(); i++) {
	
	P t = P((I*)MDAM->conn + (current.prims[i].primID()*MDAM->element_stride), (I)current.prims[i].primitivePtr);
	leaf_sequence_storage[num_stored+i] = t;
	
      }
      
      num_stored += (int)current.size();

      if(num_stored > leaf_sequence_storage.size()) { std::cout << "FAILURE: too many primitives have been stored" << std::endl; assert(false); }
      
      if ((size_t)position & 8 )  std::cout << "Uh-oh" << std::endl;
      
      return current.size() ? (NodeRef*) createLeaf(position, current.size()) : new NodeRef();
    }
    
    
    BuildState tempChildren[N];
    size_t numChildren = 1;

    std::vector<float> x,y,z;
    for(size_t i = 0; i < current.prims.size(); i++) {

      
      P t = P((I*)MDAM->conn + (current.prims[i].primID()*MDAM->element_stride), (I)current.prims[i].primitivePtr);
      
      Vec3da pnt;
      pnt = t.get_point(0, (void*)MDAM);
      x.push_back(pnt.x); y.push_back(pnt.y); z.push_back(pnt.z);
      pnt = t.get_point(1, (void*)MDAM);
      x.push_back(pnt.x); y.push_back(pnt.y); z.push_back(pnt.z);
      pnt = t.get_point(2, (void*)MDAM);
      x.push_back(pnt.x); y.push_back(pnt.y); z.push_back(pnt.z);
    }
    
    OBB bounds = OBB(&(x.front()), &(y.front()), &(z.front()), x.size());

    tempChildren[0] = current;
    for( size_t i = 1; i < numChildren; i++) {
      tempChildren[i] = BuildState(current.depth+1);
    }
    
    do {

      size_t best_child = -1;
      size_t best_size = 0;
      for (size_t i = 0; i < numChildren; i++) {
	/* ignore leaf if under the limit */
	if(tempChildren[i].prims.size() <= maxLeafSize)
	  continue;

	/* track child with largest size */
	if (tempChildren[i].prims.size() > best_size) {
	  best_size = tempChildren[i].prims.size();
	  best_child = i;
	}
      }

      /* if no child over maxLeafSize, then we're done */
      if(best_child == (size_t)-1) break;

      BuildState left(current.depth+1);
      BuildState right(current.depth+1);
      /* split the best child into left and right */
      splitFallback(tempChildren[best_child], left, right);

      /* add new children */
      tempChildren[best_child] = tempChildren[numChildren-1];
      tempChildren[numChildren-1] = left;
      tempChildren[numChildren+0]= right;
      numChildren++;
      
      /* find child with largest bounding box area */
    } while (numChildren < N);

    /* get children bounds */
    UANode* aanode = new UANode();
    
    for (size_t i = 0; i < numChildren; i++) {
      BuildSetT<PrimRef> primitives = tempChildren[i].prims;

      std::vector<float> x,y,z;
      for(size_t i = 0; i < primitives.size(); i++) {
	
	P t = P((I*)MDAM->conn + (primitives[i].primID()*MDAM->element_stride), (I)primitives[i].primitivePtr);
	
	Vec3da pnt;
	pnt = t.get_point(0, (void*)MDAM);
	x.push_back(pnt.x); y.push_back(pnt.y); z.push_back(pnt.z);
	pnt = t.get_point(1, (void*)MDAM);
	x.push_back(pnt.x); y.push_back(pnt.y); z.push_back(pnt.z);
	pnt = t.get_point(2, (void*)MDAM);
	x.push_back(pnt.x); y.push_back(pnt.y); z.push_back(pnt.z);
      }
      
      OBB b = OBB(&(x.front()), &(y.front()), &(z.front()), x.size());

      b.bump(BOX_BUMP);
      
      aanode->setBound(i, b);
      
    }

    NodeRef* node = new NodeRef((size_t)aanode, UNALIGNED_NODE);
    
    /* recurse into each child and perform reduction */
    for (size_t i = 0; i < numChildren; i++) {
#ifdef VERBOSE_MODE
      std::cout << "Recurring into CLL" << std::endl;
      std::cout << "Sending " << tempChildren[i].size() << " primitives" << std::endl;
      std::cout << *aanode << std::endl;
#endif
      NodeRef* child_node = createLargeLeaf(tempChildren[i]);
      aanode->setRef(i, *child_node);
      //      delete child_node;
    }

    depth = current.depth > depth ? current.depth : depth;
    
    return node;

  }

  
/*   NodeRef* createLargeLeaf(BuildState& current) { */
    
/*     /\* if(current.depth > maxDepth) { *\/ */
/*     /\*   std::cerr << "Maximum depth reached" << std::endl; *\/ */
/*     /\*   std::cerr << "Current depth: " << current.depth << std::endl; *\/ */
/*     /\*   std::cerr << "Maximum allowed depth: " << maxDepth << std::endl; *\/ */
/*     /\*   std::cerr << "Number of primitives remaining: " << current.size() << std::endl; *\/ */
/*     /\*   assert(false); *\/ */
/*     /\* } *\/ */

/*     if (current.size() <= maxLeafSize) { */
/*       depth = current.depth > depth ? current.depth : depth; */

/*       if(current.size() == 0 ) return new NodeRef(); */

/*       P* position = &(*(leaf_sequence_storage.begin()+num_stored)); */

/*       for( size_t i = 0; i < current.size(); i++) { */
	
/* 	P t = P((I*)MDAM->conn + (current.prims[i].primID()*MDAM->element_stride), (I)current.prims[i].primitivePtr); */
/* 	leaf_sequence_storage[num_stored+i] = t; */
	
/*       } */
      
/*       num_stored += (int)current.size(); */

/*       if(num_stored > leaf_sequence_storage.size()) { std::cout << "FAILURE: too many primitives have been stored" << std::endl; assert(false); } */
      
/*       if ((size_t)position & 8 )  std::cout << "Uh-oh" << std::endl; */
      
/*       return current.size() ? (NodeRef*) createLeaf(position, current.size()) : new NodeRef(); */
/*     } */

    
/*     BuildState tempChildren[N]; */
/*     size_t numChildren = 1; */
/*     AABB bounds = current.prims.bounds(); */
/*     tempChildren[0] = current; */
/*     for( size_t i = 1; i < numChildren; i++) { */
/*       tempChildren[i] = BuildState(current.depth+1); */
/*     } */
    
/*     do { */

/*       size_t best_child = -1; */
/*       size_t best_size = 0; */
/*       for (size_t i = 0; i < numChildren; i++) { */
/* 	/\* ignore leaf if under the limit *\/ */
/* 	if(tempChildren[i].prims.size() <= maxLeafSize) */
/* 	  continue; */

/* 	/\* track child with largest size *\/ */
/* 	if (tempChildren[i].prims.size() > best_size) { */
/* 	  best_size = tempChildren[i].prims.size(); */
/* 	  best_child = i; */
/* 	} */
/*       } */

/*       /\* if no child over maxLeafSize, then we're done *\/ */
/*       if(best_child == (size_t)-1) break; */

/*       BuildState left(current.depth+1); */
/*       BuildState right(current.depth+1); */
/*       /\* split the best child into left and right *\/ */
/*       splitFallback(tempChildren[best_child], left, right); */

/*       /\* add new children *\/ */
/*       tempChildren[best_child] = tempChildren[numChildren-1]; */
/*       tempChildren[numChildren-1] = left; */
/*       tempChildren[numChildren+0]= right; */
/*       numChildren++; */
      
/*       /\* find child with largest bounding box area *\/ */
/*     } while (numChildren < N); */

/*     /\* get children bounds *\/ */
/*     vfloat4 x_min, y_min, z_min, x_max, y_max, z_max; */
    
/*     for (size_t i = 0; i < numChildren; i++) { */
/*       AABB b; */
/*       BuildSetT<PrimRef> primitives = tempChildren[i].prims; */
      
/*       for(size_t j = 0; j < primitives.size(); j++) { */
/* 	b.update(primitives[j].lower.x, primitives[j].lower.y, primitives[j].lower.z); */
/* 	b.update(primitives[j].upper.x, primitives[j].upper.y, primitives[j].upper.z); */
/*       } */
      
/*       b.bump(BOX_BUMP); */
      
/*       x_min[i] = b.lower.x; y_min[i] = b.lower.y; z_min[i] = b.lower.z; */
/*       x_max[i] = b.upper.x; y_max[i] = b.upper.y; z_max[i] = b.upper.z; */
      
/*     } */
    
/*     /\* create node *\/ */
/*     AANode* aanode = new AANode(x_min, x_max, y_min, y_max, z_min, z_max); */
/*     NodeRef* node = new NodeRef((size_t)aanode, ALIGNED_NODE); */

    
/*     /\* recurse into each child and perform reduction *\/ */
/*     for (size_t i = 0; i < numChildren; i++) { */
/* #ifdef VERBOSE_MODE */
/*       std::cout << "Recurring into CLL" << std::endl; */
/*       std::cout << "Sending " << tempChildren[i].size() << " primitives" << std::endl; */
/*       std::cout << *aanode << std::endl; */
/* #endif */
/*       NodeRef* child_node = createLargeLeaf(tempChildren[i]); */
/*       aanode->setRef(i, *child_node); */
/*       delete child_node; */
/*     } */

/*     depth = current.depth > depth ? current.depth : depth; */
    
/*     return node; */

/*   } */


  void splitFallback(const BuildState& current, BuildState& left, BuildState& right) {
    const size_t begin_id = current.prims.prims.front().primID();
    const size_t end_id = current.prims.prims.back().primID();
    const size_t center_id = (begin_id + end_id)/2;
    
    size_t numPrimitives = current.prims.size();
    for (size_t i = 0; i < current.prims.size(); i++) {
      if (current.prims[i].primID() < center_id) {
	left.prims.push_back(current.prims[i]);
      }
      else {
	right.prims.push_back(current.prims[i]);
      }
      
    }
    
    return;
  }


  static inline bool intersect(NodeRef& node, const TravRay& ray, const vfloat4& tnear, const vfloat4& tfar, vfloat4& dist, size_t& mask) {
    if(node.isLeaf() || node.isSetLeaf() ) return false;
    if(node.isAligned()) {
      mask = intersectBox<I>(*(AANode*)node.anyNode(),ray,tnear,tfar,dist);
    }
    else if(node.isUnaligned()) {
      mask = intersectBox<I>(*(UANode*)node.anyNode(),ray,tnear,tfar,dist);
    }
    else {
      std::cout << "Could not categorize node. Not good" << std::endl;
    }
    return true;
  }

  inline void intersectRay(NodeRef root, Ray &ray) {
    TravRay vray(ray.org, ray.dir);
    intersectRay(root, ray, vray);
    return;
  }
  
  inline void intersectRay (NodeRef root, Ray &ray, TravRay &vray) {
    /* initialiez stack state */
    StackItemT<NodeRef>* stackPtr = stack+1;
    StackItemT<NodeRef>* stackEnd = stack+stackSize;
    stack[0].ptr = root;
    stack[0].dist = neg_inf;
    
    /* verify correct inputs */
    assert(ray.valid());
    assert(ray.tnear >= 0.0f);    
    
    vfloat4 ray_near = std::max(ray.tnear, 0.0);
    vfloat4 ray_far = std::max(ray.tfar, 0.0);
    
    BVHTraverser nodeTraverser = BVHTraverser();
    
    while (true) pop:
      {
	if(stackPtr == stack) break;
	stackPtr--;
	NodeRef cur = NodeRef(stackPtr->ptr);
	
	// if the ray doesn't reach this node, move to next
	if(*(float*)&stackPtr->dist > ray.tfar) { continue; }

      next:
	
	while (true)
	  {
	    size_t mask = 0; vfloat4 tNear(inf);
	    bool nodeIntersected = intersect(cur, vray, ray_near, ray_far, tNear, mask);
	    
#ifdef VERBOSE_MODE
	    AANode* curaa = (AANode*)cur.anyNode();
	    if( !cur.isEmpty() ) std::cout << curaa->bounds() << std::endl;
	    else std::cout << "EMPTY NODE" << std::endl;
	    
	    if (nodeIntersected) {
	      std::cout << "INTERIOR NODE" << std::endl;
	      std::cout << std::bitset<4>(mask) << std::endl;
	      std::cout << "Distances to hit: " << tNear << std::endl;
	      std::cout << *(AANode*)cur.anyNode() << std::endl;
	    }
	    else
	      std::cout << "LEAF NODE" << std::endl;
	    std::cout << std::endl;
#endif
	    // if no intersection, this is a leaf - check primitives
	    if (!nodeIntersected) {
	      // temporary setting of ray values
	      //	    ray.tnear = std::min(min(tNear),ray.tnear);
	      //	    ray.tfar = std::min(min(tNear),ray.tfar);
	      break; }
	    
	    // if no children were hit, pop next node
	    if (mask == 0) { goto pop; }
	    
	    nodeTraverser.traverseClosest(cur, mask, tNear, stackPtr, stackEnd);
	  }

	if ( !cur.isEmpty() ) {
	  // leaf (set distance to nearest/farthest box intersection for now)
	
	if (cur.isSetLeaf() ) {
	  // update the geom id of the travray
	  SetNode* snode = (SetNode*)cur.snode();
	  vray.setID = snode->setID;
	  if(snode->fwdID == ray.instID) {
	    vray.sense = 0;
	  }
	  else {
	    //	    assert(snode->revID == ray.instID);
	    vray.sense = 1;
	  }
	  // WILL ALSO SET SENSE HERE AT SOME POINT
	  cur = snode->ref();
	  goto next;
	  /* NodeRef setNode = cur.setLeaf(); */
	  /* intersectRay(setNode, ray, vray); */
	  /* continue; */
	}
	
	  size_t numPrims;
	  P* primIDs = (P*)cur.leaf(numPrims);
	  
	  for (size_t i = 0; i < numPrims; i++) {
	    P t = primIDs[i];
	    t.intersect(vray, ray, filter, (void*)MDAM);
	  }
	}
	
      }

    return;
  }


    static inline bool intersectNearest(NodeRef& node, const TravRay& ray, const vfloat4& tnear, const vfloat4& tfar, vfloat4& dist, size_t& mask) {
    if(node.isLeaf() || node.isSetLeaf() ) return false;
    mask = nearestOnBox<I>(*(AANode*)node.anyNode(),ray,tnear,tfar,dist);
    return true;
  }

  
  inline void intersectClosest(NodeRef root, Ray &ray) {
    TravRay vray(ray.org, ray.dir);
    intersectClosest(root, ray, vray);
    return;
  }
  
  inline void intersectClosest(NodeRef root, Ray &ray, TravRay &vray) {
        /* initialiez stack state */
    StackItemT<NodeRef>* stackPtr = stack+1;
    StackItemT<NodeRef>* stackEnd = stack+stackSize;
    stack[0].ptr = root;
    stack[0].dist = neg_inf;

    
    vfloat4 ray_near = std::max(ray.tnear, 0.0);
    vfloat4 ray_far = std::max(ray.tfar, 0.0);
    
    BVHTraverser nodeTraverser = BVHTraverser();

    while (true) pop:
      {
	if(stackPtr == stack) break;
	stackPtr--;
	NodeRef cur = NodeRef(stackPtr->ptr);
	
	// if the ray doesn't reach this node, move to next
	if(*(float*)&stackPtr->dist > ray.tfar) { continue; }

      next:
	
	while (true)
	  {
	    size_t mask = 0; vfloat4 tNear(inf);
	    bool nodeIntersected = intersectNearest(cur, vray, ray_near, ray_far, tNear, mask);

	    if(!nodeIntersected) {
	      break;
	    }

	    if (mask == 0) { goto pop; }

	    nodeTraverser.traverseClosest(cur, mask, tNear, stackPtr, stackEnd);
	  }

    	if ( !cur.isEmpty() ) {
	  // leaf (set distance to nearest/farthest box intersection for now)
	  
	  if (cur.isSetLeaf() ) {
	    // update the geom id of the travray
	    SetNode* snode = (SetNode*)cur.snode();
	    vray.setID = snode->setID;
	    if(snode->fwdID == ray.instID) {
	      vray.sense = 0;
	    }
	    else {
	      //	    assert(snode->revID == ray.instID);
	      vray.sense = 1;
	    }
	    // WILL ALSO SET SENSE HERE AT SOME POINT
	    cur = snode->ref();
	    goto next;
	    /* NodeRef setNode = cur.setLeaf(); */
	    /* intersectClosest(setNode, ray, vray); */
	    /* continue; */
	  }
	  
	  size_t numPrims;
	  P* primIDs = (P*)cur.leaf(numPrims);
	  
	  for (size_t i = 0; i < numPrims; i++) {
	    P t = primIDs[i];
	    t.closestPnt(vray, ray, (void*)MDAM);
	  }
	}
      }
    
    return;
  }
  
};

#pragma once

template<typename I>
struct SetRefT {

  SetRefT(NodeRef* node, I set_id, I fwd_id, I rev_id) : n(node), setID(set_id), fwdID(fwd_id), revID(rev_id) {}
  // pointer to the tree
  NodeRef *n;

  // set handle and sense information
  I setID, fwdID, revID;
  
};

typedef SetRefT<unsigned> SetRef;

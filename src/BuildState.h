

#pragma once

#include <set>
#include <vector>
#include "Primitive.h"


// Structure for storing the current build set
template<typename T>
struct BuildSetT{

  std::vector<T> prims;
  
  // constructors
  inline BuildSetT() {}

  inline BuildSetT(EmptyTy) {}

  inline BuildSetT( const std::set<T>& p ) {
    std::vector<T> v(prims.begin(), prims.end());
    prims = v;
  }

  inline BuildSetT(const std::vector<T>& p) { prims = p; }

  // returns a bounding box for all primitives in the BuildSet
  inline AABB bounds() {
    AABB box;
    for(unsigned int i = 0; i < prims.size(); i++ ) {
      box.update(prims[i].lower.x,prims[i].lower.y,prims[i].lower.z);
      box.update(prims[i].upper.x,prims[i].upper.y,prims[i].upper.z);
    }
    return box;
  }

  // empties out the primitive vector
  inline void clear() {
    prims.clear();
  }

  // returns the number of primitives in the BuildSet
  inline size_t size() const { return prims.size(); }

  // returns a pointer to the first primitive in the vector
  inline T* ptr () { return &(*prims.begin()); }

  // indexing operators
  inline const T& operator []( const size_t index) const { assert(index < prims.size()); return prims[index];}
  inline       T& operator []( const size_t index)       { assert(index < prims.size()); return prims[index]; }

  // wrapper for pushing primitives into the vector
  inline void push_back(const T& p) { prims.push_back(p); }
  
};


template<typename T>
struct BuildStateT {
  
  size_t depth;

  BuildSetT<T> prims;

  BuildStateT () {}

  // initialize a build state with a specified depth, but no primitives
  BuildStateT (size_t depth) : depth(depth) { prims.clear(); }

  // essentially a copy constructor
  BuildStateT (size_t depth, const std::vector<T> p) : depth(depth) { prims = BuildSetT<T>(p); }
  
  // initialize a BuildState at the top of a tree (depth is zero, but with primitives)
  BuildStateT (const std::vector<T> prims) : depth(0), prims(prims) {}

  // initialize a BuildState with c-array-like arguments
  BuildStateT (size_t depth, T* primitives, size_t numPrimitives)  : depth(depth) {
    std::vector<T> s(primitives, primitives + numPrimitives);
    prims = BuildSetT<T>(s);
  }
  
  // initialize a BuildState c-array-like arguments
  BuildStateT (T* primitives, size_t numPrimitives) : depth(0) {
    std::vector<T> s(primitives, primitives + numPrimitives);
    prims = BuildSetT<T>(s);
  }
  
  // comparators
  friend bool  operator< (const BuildStateT& a, const BuildStateT& b) { return a.prims.size() < b.prims.size(); }
  friend bool operator> (const BuildStateT& a, const BuildStateT& b) { return a.prims.size() > b.prims.size(); }

  // returns the number of primitives in the BuildState
  size_t size() const { return prims.size(); }

  // return a pointer to the first primitive in the BuildState's BuildSet
  inline T* ptr () { return prims.ptr(); }
  
};

typedef BuildSetT<BuildPrimitive> Set;
typedef BuildStateT<BuildPrimitive> BuildState;

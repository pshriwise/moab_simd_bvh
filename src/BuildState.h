

#pragma once

#include <set>
#include <vector>
#include "Primitive.h"



template<typename T>
struct SetT{
  
  inline SetT(std::set<T>& p) { std::vector<T> v(prims.begin(), prims.end());
                                  prims = v;
                              }

  inline SetT(const std::vector<T>& p) { prims = p; }
  
  inline SetT() {}

  inline SetT(EmptyTy) {}

  inline AABB bounds() {
    AABB* box = new AABB();
    for(size_t i = 0; i < prims.size(); i++ ) {
      box->update(prims[i].lower_x,prims[i].lower_y,prims[i].lower_z);
      box->update(prims[i].upper_x,prims[i].upper_y,prims[i].upper_z);
    }
    return *box;
  }

  inline void clear() { prims.clear(); }

  inline size_t size() const { return prims.size(); }

  inline T* ptr () { return &(*prims.begin()); }

  inline const T& operator []( const size_t index) const { assert(index < prims.size()); return prims[index];}
  inline       T& operator []( const size_t index)       { assert(index < prims.size()); return prims[index]; }

  inline void push_back(const T& p) { prims.push_back(p); }
  
  std::vector<T> prims;
};

typedef SetT<BuildPrimitive> Set;


template<typename T>
struct BuildStateT {
public:
  BuildStateT () {}

  BuildStateT (size_t depth) : depth(depth) { prims.clear(); }

  BuildStateT (size_t depth, const std::vector<T> p)
  : depth(depth) { prims = SetT<T>(p);
                 }

  BuildStateT (const std::vector<T> prims)
  : depth(0), prims(prims) {}

  BuildStateT (size_t depth, T* primitives, size_t numPrimitives)
  : depth(depth) { std::vector<T> s(primitives, primitives + numPrimitives);
                   prims = SetT<T>(s);
                 }

  BuildStateT (T* primitives, size_t numPrimitives)
  : depth(0) { std::vector<T> s(primitives, primitives + numPrimitives);
               prims = SetT<T>(s);
             }

  friend bool  operator< (const BuildStateT& a, const BuildStateT& b) { return a.prims.size() < b.prims.size(); }

  friend bool operator> (const BuildStateT& a, const BuildStateT& b) { return a.prims.size() > b.prims.size(); }

  size_t size() const { return prims.size(); }

  inline T* ptr () { return prims.ptr(); }

public:
  size_t depth;
  
  SetT<T> prims;
};

typedef BuildStateT<BuildPrimitive> BuildState;

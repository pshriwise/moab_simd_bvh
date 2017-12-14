

#pragma once

#include <set>
#include <vector>
#include "Primitive.h"


template<typename T>
struct BuildBuildSetT{
  
  inline BuildBuildSetT(std::set<T>& p) { std::vector<T> v(prims.begin(), prims.end());
                                  prims = v;
                              }

  inline BuildBuildSetT(const std::vector<T>& p) { prims = p; }
  
  inline BuildBuildSetT() {}

  inline BuildBuildSetT(EmptyTy) {}

  inline AABB bounds() {
    AABB box;
    for(unsigned int i = 0; i < prims.size(); i++ ) {
      box.update(prims[i].lower.x,prims[i].lower.y,prims[i].lower.z);
      box.update(prims[i].upper.x,prims[i].upper.y,prims[i].upper.z);
    }
    return box;
  }

  inline void clear() { prims.clear(); }

  inline size_t size() const { return prims.size(); }

  inline T* ptr () { return &(*prims.begin()); }

  inline const T& operator []( const size_t index) const { assert(index < prims.size()); return prims[index];}
  inline       T& operator []( const size_t index)       { assert(index < prims.size()); return prims[index]; }

  inline void push_back(const T& p) { prims.push_back(p); }
  
  std::vector<T> prims;
};

typedef BuildBuildSetT<BuildPrimitive> Set;


template<typename T>
struct BuildStateT {
public:
  BuildStateT () {}

  BuildStateT (size_t depth) : depth(depth) { prims.clear(); }

  BuildStateT (size_t depth, const std::vector<T> p)
  : depth(depth) { prims = BuildBuildSetT<T>(p);
                 }

  BuildStateT (const std::vector<T> prims)
  : depth(0), prims(prims) {}

  BuildStateT (size_t depth, T* primitives, size_t numPrimitives)
  : depth(depth) { std::vector<T> s(primitives, primitives + numPrimitives);
                   prims = BuildBuildSetT<T>(s);
                 }

  BuildStateT (T* primitives, size_t numPrimitives)
  : depth(0) { std::vector<T> s(primitives, primitives + numPrimitives);
               prims = BuildBuildSetT<T>(s);
             }

  friend bool  operator< (const BuildStateT& a, const BuildStateT& b) { return a.prims.size() < b.prims.size(); }

  friend bool operator> (const BuildStateT& a, const BuildStateT& b) { return a.prims.size() > b.prims.size(); }

  size_t size() const { return prims.size(); }

  inline T* ptr () { return prims.ptr(); }

public:
  size_t depth;
  
  BuildBuildSetT<T> prims;
};

typedef BuildStateT<BuildPrimitive> BuildState;

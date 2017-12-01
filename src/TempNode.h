
#pragma once

#include "AABB.h"

template <typename T>
struct TempNode {
  AABB box;
  std::vector<T> prims;

  inline TempNode (AABB& b, std::vector<T>& p) : box(b), prims(p) {}
  
  inline TempNode() : box(AABB((float)inf,(float)neg_inf)) {}

  void update_box() {
    for (size_t i = 0; i < prims.size() ; i++) {
      box.update(prims[i].lower.x,prims[i].lower.y,prims[i].lower.z);
      box.update(prims[i].upper.x,prims[i].upper.y,prims[i].upper.z);
    }
  }
  
  float sah_contribution() { return (0 == prims.size()) ? 0.0 : area(box)*(float)prims.size(); }

  size_t size() { return prims.size();}
  
  void clear() { box.clear(); prims.clear(); }

  void push_back(const T& p) { prims.push_back(p); }

  T& operator[](const size_t index) { return prims[index]; }

  T& front() { return prims.front(); }

  T& pop_front() { T v = prims.front();
                   prims.erase(prims.begin(), prims.begin()+1);
		   return v;
                  }  
};

typedef TempNode<BuildPrimitive> TempNodeBP;

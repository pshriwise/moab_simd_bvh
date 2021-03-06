

#pragma once

#include "constants.h"

#include "TempNode.h"

enum BVH_HEURISTIC { ENTITY_RATIO_HEURISTIC = 0,
		     SURFACE_AREA_HEURISTIC };


template<typename T>
struct BVHSettingsT {

  float (*evaluate_cost)(TempNodeT<T> tempNodes[N], const AABB &node_box, const size_t &numPrimitives);

  // constructor
  BVHSettingsT() {
    // set SAH by default
    evaluate_cost = &surface_area_heuristic;
  }

  // implementation of the entity ratio heuristic (QUAD TREE ONLY RN)
  static float entity_ratio_heuristic(TempNodeT<T> tempNodes[N], const AABB &node_box, const size_t &numPrimitives) {
    float cost = abs(abs((int)tempNodes[0].size() - (int)tempNodes[1].size()) - abs((int)tempNodes[2].size() - (int)tempNodes[3].size()));
    int total = 0;
    for(size_t i = 0; i < N; i++){ total += tempNodes[i].size(); }
    return cost /= (float)total;
  }

  // implementation of the surface area heuristic
  static float surface_area_heuristic(TempNodeT<T> tempNodes[N], const AABB &node_box, const size_t &numPrimitives) {
    float cost = 0;
    for(size_t i = 0; i < N; i++) {
      cost += tempNodes[i].sah_contribution();
    }
    return cost /= ( (float)numPrimitives * area(node_box) );
  }

  // method for switching between heuristics
  void set_heuristic(BVH_HEURISTIC h) {
    switch(h) {
    // set heuristic evaluation pointer to entity ratio
    case ENTITY_RATIO_HEURISTIC:
      {
	evaluate_cost = &entity_ratio_heuristic;
	break;
      }
    // set heuristic evaluation pointer to surface area heuristic
    case SURFACE_AREA_HEURISTIC:
      {
	evaluate_cost = &surface_area_heuristic;
	break;
      }
    // if an invalid heuristic type is specified, report to user and do nothing
    default:
      std::cout << "INVALID HEURISTIC" << std::endl;
    }
  }
};

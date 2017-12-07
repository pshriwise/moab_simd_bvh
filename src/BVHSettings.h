

#pragma once

#include "constants.h"

#include "TempNode.h"

enum BVH_HEURISTIC { ENTITY_RATIO_HEURISTIC = 0,
		     SURFACE_AREA_HEURISTIC };


template<typename T>
struct BVHSettings {

  BVHSettings() {
    evaluate_cost = &surface_area_heuristic;
  }
  		       
  float (*evaluate_cost)(TempNode<T> tempNodes[N]);


  static float entity_ratio_heuristic(TempNode<T> tempNodes[N]) {
    float cost = abs(abs(tempNodes[0].size() - tempNodes[1].size()) - abs(tempNodes[2].size() - tempNodes[3].size()));
    int total = 0;
    for(size_t i = 0; i < N; i++){ total += tempNodes[i].size(); }
    return cost /= (float)total;
  }

  static float surface_area_heuristic(TempNode<T> tempNodes[N]) {
    float cost = 0;
    int total = 0;
    AABB node_box;
    for(size_t i = 0; i < N; i++) {
      cost += tempNodes[i].sah_contribution();
      total += tempNodes[i].size();
      node_box.update(tempNodes[i].box);
    }
    return cost /= ( (float)total * area(node_box) );
  }

  void set_heuristic(BVH_HEURISTIC h) {

    switch(h) {
    case ENTITY_RATIO_HEURISTIC:
      {
	evaluate_cost = &entity_ratio_heuristic;
	break;
      }
    case SURFACE_AREA_HEURISTIC:
      {
	evaluate_cost = &surface_area_heuristic;
	break;
      }
    default:
      std::cout << "INVALID HEURISTIC" << std::endl;
    }
    
  }

};



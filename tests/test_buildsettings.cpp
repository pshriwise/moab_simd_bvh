
#include "testutil.hpp"
#include "test_files.h"

#include "BVHSettings.h"
#include "TriangleRef.h"

int main(int argc, char** argv) {

  BVHSettings<int> settings;

  settings.set_heuristic(ENTITY_RATIO_HEURISTIC);

  TempNode<int> tempNodes[4];

  float cost;
  float expected_cost;
  // setup the nodes for a test
  tempNodes[0].prims.resize(25);
  tempNodes[1].prims.resize(25);
  tempNodes[2].prims.resize(25);
  tempNodes[3].prims.resize(25);

  AABB dummy_box;
  size_t numPrims = 100;
  
  expected_cost = 0.0;
  cost = settings.entity_ratio_heuristic(tempNodes, dummy_box, numPrims);
  CHECK_REAL_EQUAL(expected_cost, cost, 0.0f);
  cost = settings.evaluate_cost(tempNodes, dummy_box, numPrims);
  CHECK_REAL_EQUAL(expected_cost, cost, 0.0f);

  // next test
  tempNodes[0].prims.resize(30);
  tempNodes[1].prims.resize(20);
  tempNodes[2].prims.resize(25);
  tempNodes[3].prims.resize(25);
  
  cost = settings.entity_ratio_heuristic(tempNodes, dummy_box, numPrims);
  expected_cost = 0.1f;
  CHECK_REAL_EQUAL(expected_cost, cost, 0.0f);

  //change to Surface Area Heuristic
  settings.set_heuristic(SURFACE_AREA_HEURISTIC);

  
  // setup the nodes for a test
  tempNodes[0].prims.resize(25);
  tempNodes[1].prims.resize(25);
  tempNodes[2].prims.resize(25);
  tempNodes[3].prims.resize(25);

  new (&tempNodes[0].box) AABB(-20.0f, -10.0f);
  new (&tempNodes[1].box) AABB(-10.0f,   0.0f);
  new (&tempNodes[2].box) AABB(  0.0f,  10.0f);
  new (&tempNodes[3].box) AABB( 10.0f,  20.0f);

  AABB node_box;
  node_box.extend(tempNodes[0].box);
  node_box.extend(tempNodes[1].box);
  node_box.extend(tempNodes[2].box);
  node_box.extend(tempNodes[3].box);

  expected_cost = 0.0625f;
  cost = settings.evaluate_cost(tempNodes, node_box, numPrims);
 
  CHECK_REAL_EQUAL(expected_cost, cost, 0.0f);

  // vary entities
  tempNodes[0].prims.resize(30);
  tempNodes[1].prims.resize(20);
  tempNodes[2].prims.resize(25);
  tempNodes[3].prims.resize(25);

  new (&tempNodes[0].box) AABB(-20.0f, -10.0f);
  new (&tempNodes[1].box) AABB(-10.0f,   0.0f);
  new (&tempNodes[2].box) AABB(  0.0f,  10.0f);
  new (&tempNodes[3].box) AABB( 10.0f,  20.0f);

  expected_cost = 0.0625f;
  cost = settings.evaluate_cost(tempNodes, node_box, numPrims);  
  CHECK_REAL_EQUAL(expected_cost, cost, 0.0f);
  
  return 0;
}

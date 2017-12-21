
#include "testutil.hpp"
#include "test_files.h"

#include "BVHSettings.h"
#include "TriangleRef.h"

int main(int argc, char** argv) {

  BVHSettingsT<int> settings;

  TempNodeT<int> tempNodes[4];

  float cost;
  float expected_cost;
  AABB dummy_box;
  size_t numPrims = 100;

  /* Entity Ratio Heuristic Tests */
  settings.set_heuristic(ENTITY_RATIO_HEURISTIC);
  
  // test with an even split
  tempNodes[0].prims.resize(25);
  tempNodes[1].prims.resize(25);
  tempNodes[2].prims.resize(25);
  tempNodes[3].prims.resize(25);

  expected_cost = 0.0;
  cost = settings.entity_ratio_heuristic(tempNodes, dummy_box, numPrims);
  CHECK_REAL_EQUAL(expected_cost, cost, 0.0f);
  cost = settings.evaluate_cost(tempNodes, dummy_box, numPrims);
  CHECK_REAL_EQUAL(expected_cost, cost, 0.0f);

  // test for a slightly skewed split
  tempNodes[0].prims.resize(30);
  tempNodes[1].prims.resize(20);
  tempNodes[2].prims.resize(25);
  tempNodes[3].prims.resize(25);
  
  cost = settings.entity_ratio_heuristic(tempNodes, dummy_box, numPrims);
  expected_cost = 0.1f;
  CHECK_REAL_EQUAL(expected_cost, cost, 0.0f);

  /* Surface Area Heuristic Tests */
  settings.set_heuristic(SURFACE_AREA_HEURISTIC);
  
  // test an even split
  tempNodes[0].prims.resize(25);
  tempNodes[1].prims.resize(25);
  tempNodes[2].prims.resize(25);
  tempNodes[3].prims.resize(25);

  // set the box sizes
  new (&tempNodes[0].box) AABB(-20.0f, -10.0f);
  new (&tempNodes[1].box) AABB(-10.0f,   0.0f);
  new (&tempNodes[2].box) AABB(  0.0f,  10.0f);
  new (&tempNodes[3].box) AABB( 10.0f,  20.0f);

  // mimic a parent box
  AABB node_box;
  node_box.extend(tempNodes[0].box);
  node_box.extend(tempNodes[1].box);
  node_box.extend(tempNodes[2].box);
  node_box.extend(tempNodes[3].box);
  
  // test
  expected_cost = 0.0625f;
  cost = settings.evaluate_cost(tempNodes, node_box, numPrims);
  CHECK_REAL_EQUAL(expected_cost, cost, 0.0f);

  // test a slightly skewed split in number of entities
  tempNodes[0].prims.resize(30);
  tempNodes[1].prims.resize(20);
  tempNodes[2].prims.resize(25);
  tempNodes[3].prims.resize(25);

  new (&tempNodes[0].box) AABB(-20.0f, -10.0f);
  new (&tempNodes[1].box) AABB(-10.0f,   0.0f);
  new (&tempNodes[2].box) AABB(  0.0f,  10.0f);
  new (&tempNodes[3].box) AABB( 10.0f,  20.0f);

  //test
  expected_cost = 0.0625f;
  cost = settings.evaluate_cost(tempNodes, node_box, numPrims);  
  CHECK_REAL_EQUAL(expected_cost, cost, 0.0f);

  // test a slightly skewed split in number of entities
  tempNodes[0].prims.resize(30);
  tempNodes[1].prims.resize(20);
  tempNodes[2].prims.resize(25);
  tempNodes[3].prims.resize(25);

  // with different sized boxes
  new (&tempNodes[0].box) AABB(-15.0f, -10.0f);
  new (&tempNodes[1].box) AABB(-10.0f,   0.0f);
  new (&tempNodes[2].box) AABB(  0.0f,  10.0f);
  new (&tempNodes[3].box) AABB( 10.0f,  15.0f);

  node_box.clear();
  node_box.extend(tempNodes[0].box);
  node_box.extend(tempNodes[1].box);
  node_box.extend(tempNodes[2].box);
  node_box.extend(tempNodes[3].box);
  
  //test
  expected_cost = 0.065277778f;
  cost = settings.evaluate_cost(tempNodes, node_box, numPrims);
  CHECK_REAL_EQUAL(expected_cost, cost, 0.0f);

  return 0;
}

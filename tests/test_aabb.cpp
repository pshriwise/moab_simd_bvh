#include <stdio.h>

#include "AABB.h"
#include "testutil.hpp"


void constructor_tests();
void point_contain_tests();
void extend_tests();
void merge_tests();
void property_tests();
int main( int argc, char** argv) {

  // test AABB constructor
  constructor_tests();
  // test point containment method
  point_contain_tests();
  // test box extension using another box
  extend_tests();
  // box merging tests
  merge_tests();
  // testing various property calculations for the box
  property_tests();
  
  return 0;

}

void constructor_tests() {
  //test-box values
  float x_min = 0.0, y_min = 1.0, z_min = 2.0,
        x_max = 3.0, y_max = 4.0, z_max = 5.0;

  float test_extents[6] = {x_min, y_min, z_min, x_max, y_max, z_max};
  float test_lower[3] = {x_min, y_min, z_min};
  float test_upper[3] = {x_max, y_max, z_max};

  // create AABB using an array
  AABB box = AABB(test_extents);
  CHECK_VECREAL_EQUAL(Vec3f(test_lower), box.lower);
  CHECK_VECREAL_EQUAL(Vec3f(test_upper), box.upper);
  
  // test copy constructor
  AABB box1 = box;
  // test that values equal original box
  CHECK_VECREAL_EQUAL(box.lower, box1.lower);
  CHECK_VECREAL_EQUAL(box.upper, box1.upper);
  // test that extents equal oritinal values
  CHECK_VECREAL_EQUAL(Vec3f(test_lower), box1.lower);
  CHECK_VECREAL_EQUAL(Vec3f(test_upper), box1.upper);

  // test 'explicit' min/max constructor
  AABB box2 = AABB(x_min, y_min, z_min, x_max, y_max, z_max);
  // test that values are correct
  CHECK_REAL_EQUAL(x_min, box2.lower[0], 0.0);
  CHECK_REAL_EQUAL(y_min, box2.lower[1], 0.0);
  CHECK_REAL_EQUAL(z_min, box2.lower[2], 0.0);
  CHECK_REAL_EQUAL(x_max, box2.upper[0], 0.0);
  CHECK_REAL_EQUAL(y_max, box2.upper[1], 0.0);
  CHECK_REAL_EQUAL(z_max, box2.upper[2], 0.0);

  AABB box3 = AABB(Vec3f(test_lower), Vec3f(test_upper));
  CHECK_VECREAL_EQUAL(Vec3f(test_lower), box3.lower);
  CHECK_VECREAL_EQUAL(Vec3f(test_upper), box3.upper);
  
  // test empty constructor
  AABB empty = AABB();
}

void point_contain_tests() {
  // test-box values
  float x_min = 0.0, y_min = 1.0, z_min = 2.0,
        x_max = 3.0, y_max = 4.0, z_max = 5.0;
  // create test box
  AABB box = AABB(x_min, y_min, z_min, x_max, y_max, z_max);

  //create test point inside box
  Vec3f p(1.5,2.0, 4.0);
  CHECK(inside(box,p));

  //update x position to be outside limits
  p[0] = 3.5;
  CHECK(!inside(box,p));

  // add point just outside box in double,
  // should be inside for single precision
  p = Vec3f(3.0000000000001, 3.0, 3.0);
  CHECK(inside(box,p));

  // add point just outside box in single precision,
  // should be outside for single precision
  p = Vec3f(3.0001, 3.0, 3.0);
  CHECK(!inside(box,p));

}

void extend_tests() {

  // create test boxes
  AABB box1 = AABB(0.0, 0.0, 0.0,  5.0,  5.0,  5.0);
  AABB box2 = AABB(5.0, 5.0, 5.0, 10.0, 10.0, 10.0);

  //extend a copy of box1 to match box2
  AABB result;

  // extend box1 to box2
  result = box1.extend(box2);
  // the resulting box should have the same
  // values as box1's lower extents
  CHECK_VECREAL_EQUAL(box1.lower, result.lower);
  // the resulting box should have the same
  // values as box2's upper extents
  CHECK_VECREAL_EQUAL(box2.upper, result.upper);

  // extend box2 to box1
  result = box2.extend(box1);
  // the resulting box should have the same
  // values as box1's lower extents
  CHECK_VECREAL_EQUAL(box1.lower, result.lower);
  // the resulting box should have the same
  // values as box2's upper extents
  CHECK_VECREAL_EQUAL(box2.upper, result.upper);  
}

void merge_tests() {

    // create test boxes
  AABB box1 = AABB(0.0, 0.0, 0.0,  5.0,  5.0,  5.0);
  AABB box2 = AABB(5.0, 5.0, 5.0, 10.0, 10.0, 10.0);

  //extend a copy of box1 to match box2
  AABB result;

  result = merge(box1,box2);
  // the resulting box should have the same
  // values as box1's lower extents
  CHECK_VECREAL_EQUAL(box1.lower, result.lower);
  // the resulting box should have the same
  // values as box2's upper extents
  CHECK_VECREAL_EQUAL(box2.upper, result.upper);

  
}

void property_tests() {
    // create test boxes
  AABB box = AABB(5.0, -5.0, -3.0, 10.0, 15.0, 4.0);

  Vec3f expected_center(7.5, 5.0, 0.5);
  CHECK_VECREAL_EQUAL(expected_center, box.center());

  Vec3f expected_center2(15.0, 10.0, 1.0);
  CHECK_VECREAL_EQUAL(expected_center2, box.center2());

  Vec3f expected_size(5.0, 20.0, 7.0);
  CHECK_VECREAL_EQUAL(expected_size, box.size());

  float expected_halfArea = 275.0;
  CHECK_REAL_EQUAL(expected_halfArea, halfArea(box), 0.0);
}


#include <iostream>

#include "OBB.h"
#include "AABB.h"
#include "testutil.hpp"

// test function signatures
void constructor_tests();
void point_contain_tests();
void property_tests();
void ray_intersection_tests();
void construction_tests();

int main( int argc, char** argv) {

  // test AABB constructor
  constructor_tests();
  // test point containment method
  point_contain_tests();
  // testing various property calculations for the box
  property_tests();
  // test intersections with box
  ray_intersection_tests();
  // test proper construction
  construction_tests();
  
  return 0;
}

void constructor_tests() {
  
  //test-box values
  Vec3fa center(1.0f, 1.0f, 1.0f);
  Vec3fa ax0(0.0f, 1.0f, 0.0f);
  Vec3fa ax1(1.0f, 0.0f, 0.0f);
  Vec3fa ax2(0.0f, 0.0f, 1.0f);
  Vec3fa size(2.0f);
  // create AABB using an array
  OBB box = OBB(center, size, ax0, ax1, ax2);

  // test that values are correct
  CHECK_VECREAL_EQUAL(center, box.center());
  CHECK_VECREAL_EQUAL(ax0, box.transform.row0());
  CHECK_VECREAL_EQUAL(ax1, box.transform.row1());
  CHECK_VECREAL_EQUAL(ax2, box.transform.row2());

  // test empty constructor
  OBB empty = OBB();
  // box should be invalid when created
  // using an empty constructor
  CHECK(!empty.isValid());

  return;
}

void point_contain_tests() {
  
  // test-box values
  float x_min = 0.0, y_min = 1.0, z_min = 2.0,
        x_max = 3.0, y_max = 4.0, z_max = 5.0;

  Vec3fa center((x_min + x_max) / 2.0f,
		(y_min + y_max) / 2.0f,
		(z_min + z_max) / 2.0f);
  Vec3fa ax0 ( (x_max - x_min) / 2.0f, 0.0f, 0.0f );
  Vec3fa ax1 ( 0.0f, (y_max - y_min) / 2.0f, 0.0f );
  Vec3fa ax2 ( 0.0f, 0.0f, (z_max - z_min) / 2.0f );
  Vec3fa size( x_max - x_min, y_max - y_min, z_max - z_min);
    
  // create test box
  OBB box = OBB(center, size, ax0, ax1, ax2);
  
  //  AABB box = AABB(x_min, y_min, z_min, x_max, y_max, z_max);

  //create test point inside box
  Vec3fa p(1.5,2.0, 4.0);
  CHECK(inside(box,p));

  //update x position to be outside limits
  p[0] = 3.5;
  CHECK(!inside(box,p));

  // add point just outside box in double,
  // should be inside for single precision
  p = Vec3fa(3.0000000000001, 3.0, 3.0);
  CHECK(inside(box,p));

  // add point just outside box in single precision,
  // should be outside for single precision
  p = Vec3fa(3.0001, 3.0, 3.0);
  CHECK(!inside(box,p));

  return;
}

void property_tests() {
  
  // create test boxes

  // test-box values
  float x_min = 0.0, y_min = 1.0, z_min = 2.0,
        x_max = 3.0, y_max = 4.0, z_max = 5.0;

  Vec3fa center((x_min + x_max) / 2.0f,
		(y_min + y_max) / 2.0f,
		(z_min + z_max) / 2.0f);
  Vec3fa ax0 ( (x_max - x_min) / 2.0f, 0.0f, 0.0f );
  Vec3fa ax1 ( 0.0f, (y_max - y_min) / 2.0f, 0.0f );
  Vec3fa ax2 ( 0.0f, 0.0f, (z_max - z_min) / 2.0f );
  Vec3fa size( x_max - x_min, y_max - y_min, z_max - z_min);

  OBB box = OBB(center, size, ax0, ax1, ax2);
  
  // check center
  Vec3fa expected_center(1.5, 2.5, 3.5);
  CHECK_VECREAL_EQUAL(expected_center, box.center());

  // check center2
  Vec3fa expected_center2(3.0, 5.0, 7.0);
  CHECK_VECREAL_EQUAL(expected_center2, box.center2());

  // check size
  Vec3fa expected_size(3.0, 3.0, 3.0);
  CHECK_VECREAL_EQUAL(expected_size, box.size());

  // check halfarea
  float expected_halfArea = 27.0;
  CHECK_REAL_EQUAL(expected_halfArea, halfArea(box), 0.0f);

  // check actual area
  float expected_area = 54.0;
  CHECK_REAL_EQUAL(expected_area, area(box), 0.0f);

  float expected_vol = 27.0;
  CHECK_REAL_EQUAL(expected_vol, volume(box), 0.0f);
  
  return;
}

void ray_intersection_tests() {

    // test-box values
  float x_min = 0.0, y_min = 1.0, z_min = 2.0,
        x_max = 3.0, y_max = 4.0, z_max = 5.0;

  Vec3fa center((x_min + x_max) / 2.0f,
		(y_min + y_max) / 2.0f,
		(z_min + z_max) / 2.0f);
  Vec3fa ax0 ( (x_max - x_min) / 2.0f, 0.0f, 0.0f );
  Vec3fa ax1 ( 0.0f, (y_max - y_min) / 2.0f, 0.0f );
  Vec3fa ax2 ( 0.0f, 0.0f, (z_max - z_min) / 2.0f );
  Vec3fa size( x_max - x_min, y_max - y_min, z_max - z_min);

  // create test box
  OBB box = OBB(center, size, ax0, ax1, ax2);

  // create test ray
  RayT<Vec3fa,float,int> ray;
  ray.org = Vec3fa(9.0f, 2.0f, 3.0f);
  ray.dir = Vec3fa(-1.0f, 0.0f, 0.0f);
  ray.tfar = inf;

  // check ray toward box intersects
  CHECK(ray_intersection(box, ray));

  // reverse ray direction
  ray.dir = -ray.dir;

  // check ray away from box does not intersect
  CHECK(!ray_intersection(box, ray));

  // move ray origin into box
  ray.org.x = 2.0f;

  // check for intersection
  CHECK(ray_intersection(box, ray));

  // move ray origin just outside of the box
  ray.org.x = 3.0001f;

  // check for no intersection
  CHECK(!ray_intersection(box, ray));

  //align ray origin with the edge
  ray.org.x = 2.9f;

  // fire ray along box edge
  ray.dir = Vec3fa(0.0f, 1.0f, 0.0f);

  // check that the ray intersects the box
  CHECK(ray_intersection(box, ray));
  
  return;
}

float nrandf() { return (float) (rand()/RAND_MAX); }

void construction_tests() {
  // generate a random set of points
  std::vector<float> x, y, z;

  int num_pnts = 100;
  
  for(int i = 0; i < num_pnts; i ++){
    x.push_back(nrandf());
    y.push_back(nrandf());
    z.push_back(nrandf());
  }

  OBB box(&x.front(), &y.front(), &z.front(), num_pnts);

  for(int i = 0; i < num_pnts; i++) {
    Vec3fa pnt(x[i], y[i], z[i]);
    CHECK(box.point_in_box(pnt));
  }
  
  
  return;
}

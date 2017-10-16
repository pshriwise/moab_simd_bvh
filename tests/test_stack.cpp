
#include <stdio.h>

#include "testutil.hpp"
#include "Stack.h"
#include "Primitive.h"
#include "Node.h"
#include "Builder.h"

void test_xchg();
void test_sort();


int main(int argc, char** argv) {

  test_xchg();

  test_sort();
  
  return 0;
}


void test_xchg() {

  NodeRef n1(5), n2(10);
  
  StackItemT<NodeRef> s1, s2;

  s1.ptr = n1;
  s2.ptr = n2;

  CHECK_EQUAL(n1, s1.ptr);
  CHECK_EQUAL(n2, s2.ptr);

  StackItemT<NodeRef>::xchg(s1, s2);

  CHECK_EQUAL(n2, s1.ptr);
  CHECK_EQUAL(n1, s2.ptr);

}

void test_sort() {

  BuildPrimitive tp1, tp2, tp3;

  tp1 = BuildPrimitive(0.0, 0.0, 0.0, 0,
		      1.0, 1.0, 1.0, 1);
  tp2 = BuildPrimitive(1.0, 1.0, 1.0, 1,
		      2.0, 2.0, 2.0, 2);
  tp3 = BuildPrimitive(2.0, 2.0, 2.0, 2,
		      3.0, 3.0, 3.0, 3);
  

  BuildPrimitiveBVH BVH;
  
  NodeRef* n1 = (NodeRef*)BVH.createLeaf(&tp1, 1);
  NodeRef* n2 = (NodeRef*)BVH.createLeaf(&tp2, 1);
  NodeRef* n3 = (NodeRef*)BVH.createLeaf(&tp3, 1);
  
  StackItemT<NodeRef> s1, s2, s3;

  s1.ptr = *n1;
  s2.ptr = *n2;
  s3.ptr = *n3;

  s1.dist = 11.0486;
  s2.dist = 10.0039;
  s3.dist = 9.69619;

  for(unsigned int i = 0; i < 1e3 ; i++)
    {
      sort(s1, s2, s3);

      CHECK_EQUAL(*n3, s1.ptr);
      CHECK_EQUAL(*n2, s2.ptr);
      CHECK_EQUAL(*n1, s3.ptr);

      size_t dum;

      BuildPrimitive* p1 = (BuildPrimitive*)n1->leaf(dum);
      CHECK( tp1 == *p1);
      BuildPrimitive* p2 = (BuildPrimitive*)n2->leaf(dum);
      CHECK( tp2 == *p2);
      BuildPrimitive* p3 = (BuildPrimitive*)n3->leaf(dum);
      CHECK( tp3 == *p3);
      
    }

  
}

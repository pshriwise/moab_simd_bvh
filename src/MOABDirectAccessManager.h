
#pragma once


struct MOABDirectAccessManager {

  inline MOABDirectAccessManager(double *xPtr, double *yPtr, double *zPtr, int num_vertices, long unsigned int first_element, moab::EntityHandle *connPointer, int num_elements, int element_stride) :
    xPtr(xPtr),
    yPtr(yPtr),
    zPtr(zPtr),
    num_vertices(num_vertices),
    first_element(first_element),
    conn(connPointer),
    num_elements(num_elements),
    element_stride(element_stride){}

  int num_elements;
  int num_vertices;

  int element_stride;
  
  long unsigned int first_element;
  
  moab::EntityHandle* conn;
  
  double *xPtr, *yPtr, *zPtr;

};

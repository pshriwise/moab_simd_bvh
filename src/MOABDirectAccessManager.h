
#pragma once


struct MOABDirectAccessManager {

  inline MOABDirectAccessManager(moab::Interface* mbi, double *xPtr, double *yPtr, double *zPtr, int num_vertices, moab::EntityHandle first_element, moab::EntityHandle *connPointer, int num_elements, int element_stride) :
    xPtr(xPtr),
    yPtr(yPtr),
    zPtr(zPtr),
    num_vertices(num_vertices),
    first_element(first_element),
    conn(connPointer),
    num_elements(num_elements),
    element_stride(element_stride),
    MOAB_instance(mbi){}

  int num_elements;
  int num_vertices;

  int element_stride;
  
  moab::EntityHandle first_element;
  
  moab::EntityHandle* conn;

  moab::Interface* MOAB_instance;
  
  double *xPtr, *yPtr, *zPtr;

};

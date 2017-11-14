
#pragma once


struct MOABDirectAccessManager {

  inline MOABDirectAccessManager(long unsigned int id, double *xPtr, double *yPtr, double *zPtr, void* connPointer) :
    id(id),
    conn(connPointer),
    xPtr(xPtr),
    yPtr(yPtr),
    zPtr(zPtr) {}

  long unsigned int id;
  
  void* conn;
  
  double *xPtr, *yPtr, *zPtr;

};

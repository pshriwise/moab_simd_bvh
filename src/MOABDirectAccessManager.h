
#pragma once


struct MOABDirectAccessManager {

  inline MOABDirectAccessManager(double *xPtr, double *yPtr, double *zPtr, void* connPointer) :
    conn(connPointer),
    xPtr(xPtr),
    yPtr(yPtr),
    zPtr(zPtr) {}
												
  void* conn;
  
  double *xPtr, *yPtr, *zPtr;

};



#pragma once



struct BuildPrimitive {
  float lower_x, lower_y, lower_z;
  int sceneID;
  float upper_x, upper_y, upper_z;
  int primID;

  BuildPrimitive () {}
  
  BuildPrimitive (float lx, float ly, float lz, int sID, float ux, float uy, float uz, int pID) :
  lower_x(lx), lower_y(ly), lower_z(lz), sceneID(sID), upper_x(ux), upper_y(uy), upper_z(uz), primID(pID) {}
  
  friend bool operator< (const BuildPrimitive& a, const BuildPrimitive& b) { return a.primID < b.primID; }

  friend bool operator!= (const BuildPrimitive& a, const BuildPrimitive& b) { return a.primID != b.primID; }

  AABB box() { return AABB(lower_x, lower_y, lower_z, upper_x, upper_y, upper_z); }
  
  Vec3fa center() const { return (Vec3fa(lower_x,lower_y,lower_z)+Vec3fa(upper_x,upper_y,upper_z))/2.0f; }
};

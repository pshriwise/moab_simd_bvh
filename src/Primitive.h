

#pragma once



struct BuildPrimitive {
  float lower_x, lower_y, lower_z;
  int sceneID;
  float upper_x, upper_y, upper_z;
  int primID;

  friend bool operator< (const BuildPrimitive& a, const BuildPrimitive& b) { return a.primID < b.primID; }

  friend bool operator!= (const BuildPrimitive& a, const BuildPrimitive& b) { return a.primID != b.primID; }

  Vec3fa center() const { return (Vec3fa(lower_x,lower_y,lower_z)+Vec3fa(upper_x,upper_y,upper_z))/2.0f; }
};

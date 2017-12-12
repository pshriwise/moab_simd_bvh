#pragma once

// #define VERBOSE_MODE

#include "Primitive.h"

#include "moab/Core.hpp"
#include "moab/CartVect.hpp"
#include "moab/GeomUtil.hpp"
#include "MOABDirectAccessManager.h"

struct TriangleRef : public BuildPrimitive {
  using::BuildPrimitive::lower;
  using::BuildPrimitive::upper;
  
  //  inline TriangleRef(long unsigned int tri_handle): eh(tri_handle){ set_primID(eh); }

  inline TriangleRef(moab::EntityHandle tri_handle, moab::Interface* mbi): eh(tri_handle) {
    set_bounds(mbi);
    set_primID(eh);
    set_sceneID((size_t)mbi);
  }

  friend bool operator< (const TriangleRef& a, const TriangleRef& b) { return a.eh < b.eh; }

  friend bool operator!= (const TriangleRef& a, const TriangleRef& b) { return a.eh != b.eh; }

  friend bool operator== (const TriangleRef& a, const TriangleRef& b) { return a.eh == b.eh; }

  inline void set_bounds(moab::Interface* mbi) {
    
    std::vector<moab::EntityHandle> conn;
    moab::ErrorCode rval = mbi->get_connectivity(&eh, 1, conn);
    MB_CHK_SET_ERR_RET(rval, "Failed to get triangle connectivity.");

    assert(conn.size() == 3);

    moab::CartVect coords[3];
    
    rval = mbi->get_coords(&(conn[0]), 3, coords[0].array() );
    MB_CHK_SET_ERR_RET(rval, "Failed to get triangle vert coords");

    
    const float round_down = 1.0f-2.0f*float(ulp); // FIXME: use per instruction rounding for AVX512
    const float round_up   = 1.0f+2.0f*float(ulp);

    float bump = 5e-03;
    
    upper.x = std::max(coords[0][0],std::max(coords[1][0], coords[2][0]));
    upper.x += bump;
    upper.y = std::max(coords[0][1],std::max(coords[1][1], coords[2][1]));
    upper.y += bump;
    upper.z = std::max(coords[0][2],std::max(coords[1][2], coords[2][2]));
    upper.z += bump;
    
    lower.x = std::min(coords[0][0],std::min(coords[1][0], coords[2][0]));
    lower.x -= bump;
    lower.y = std::min(coords[0][1],std::min(coords[1][1], coords[2][1]));
    lower.y -= bump;
    lower.z = std::min(coords[0][2],std::min(coords[1][2], coords[2][2]));
    lower.z -= bump;
    	
  }
  
  template <typename R>
  inline bool intersect(R &ray) {

    moab::ErrorCode rval;

    moab::Interface* mbi = (moab::Interface*)sceneID();
    
    std::vector<moab::EntityHandle> conn;
    rval = mbi->get_connectivity(&eh, 1, conn);
    MB_CHK_SET_ERR_CONT(rval, "Failed to get triangle connectivity.");

    assert(conn.size() == 3);

    moab::CartVect coords[3];
    
    rval = mbi->get_coords(&(conn[0]), 3, coords[0].array() );
    MB_CHK_SET_ERR_CONT(rval, "Failed to get triangle vert coords");
    
    moab::CartVect origin, direction;

    origin = moab::CartVect(ray.org.x, ray.org.y, ray.org.z);
    direction = moab::CartVect(ray.dir.x, ray.dir.y, ray.dir.z);
    double dist;
    double huge_val = 1E37;
    double* nonneg_ray_len = &huge_val;

    bool hit = moab::GeomUtil::plucker_ray_tri_intersect(coords,
							 origin,
							 direction,
							 dist,
							 nonneg_ray_len);

#ifdef VERBOSE_MODE
    std::cout << *this;
    std::cout << "Triangle Vert Coords: " << coords[0] << coords[1] << coords[2] << std::endl;
    if (hit) std::cout << "Hit found at distance: " << dist << std::endl;
    std::cout << std::endl;
#endif

    if (hit && dist < ray.tfar) {
      ray.primID = eh;
      ray.tfar = dist;
    }

    return hit;
  }
  
  long unsigned int eh;
  
};
  
template<typename I>  
struct MBTriangleRefT {

  size_t i1, i2, i3;
  I eh;

  inline MBTriangleRefT() {}
  
  inline MBTriangleRefT(moab::EntityHandle* conn_ptr, moab::EntityHandle id) : eh(id) {
    i1 = *(conn_ptr)-1;
    i2 = *(conn_ptr + 1)-1;
    i3 = *(conn_ptr + 2)-1;
  }

  inline void get_bounds(Vec3fa& lower, Vec3fa& upper, void* mesh_ptr = NULL) {

    if( !mesh_ptr ) MB_CHK_SET_ERR_RET(moab::MB_FAILURE, "No Mesh Pointer");

    MOABDirectAccessManager *mdam = (MOABDirectAccessManager*) mesh_ptr;

    moab::CartVect coords[3];

    coords[0] = moab::CartVect(mdam->xPtr[i1], mdam->yPtr[i1], mdam->zPtr[i1]);
    coords[1] = moab::CartVect(mdam->xPtr[i2], mdam->yPtr[i2], mdam->zPtr[i2]);
    coords[2] = moab::CartVect(mdam->xPtr[i3], mdam->yPtr[i3], mdam->zPtr[i3]);

    /* moab::Interface* mbi = (moab::Interface*) mesh_ptr; */
   
    /* moab::CartVect coords[3]; */
    
    /* moab::ErrorCode rval = mbi->get_coords(eh, 3, coords[0].array() ); */
    /* MB_CHK_SET_ERR_RET(rval, "Failed to get triangle vert coords"); */


    const float round_down = 1.0f-2.0f*float(ulp); // FIXME: use per instruction rounding for AVX512
    const float round_up   = 1.0f+2.0f*float(ulp);

    float bump = 5e-03;
    
    upper.x = std::max(coords[0][0],std::max(coords[1][0], coords[2][0]));
    upper.x += bump;
    upper.y = std::max(coords[0][1],std::max(coords[1][1], coords[2][1]));
    upper.y += bump;
    upper.z = std::max(coords[0][2],std::max(coords[1][2], coords[2][2]));
    upper.z += bump;
    
    lower.x = std::min(coords[0][0],std::min(coords[1][0], coords[2][0]));
    lower.x -= bump;
    lower.y = std::min(coords[0][1],std::min(coords[1][1], coords[2][1]));
    lower.y -= bump;
    lower.z = std::min(coords[0][2],std::min(coords[1][2], coords[2][2]));
    lower.z -= bump;
    	
  }

  inline bool intersect(const TravRay& tray, dRay &ray, void* mesh_ptr = NULL) {


    if( !mesh_ptr ) MB_CHK_SET_ERR_CONT(moab::MB_FAILURE, "No Mesh Pointer");

    MOABDirectAccessManager* mdam = (MOABDirectAccessManager*) mesh_ptr;

    moab::CartVect coords[3];

    coords[0] = moab::CartVect(mdam->xPtr[i1], mdam->yPtr[i1], mdam->zPtr[i1]);
    coords[1] = moab::CartVect(mdam->xPtr[i2], mdam->yPtr[i2], mdam->zPtr[i2]);
    coords[2] = moab::CartVect(mdam->xPtr[i3], mdam->yPtr[i3], mdam->zPtr[i3]);

    /* moab::ErrorCode rval; */
    
    /* moab::CartVect coords[3]; */
    
    /* rval = mbi->get_coords(eh, 3, coords[0].array() ); */
    /* MB_CHK_SET_ERR_CONT(rval, "Failed to get triangle vert coords"); */
    
    moab::CartVect origin, direction;

    origin = moab::CartVect(ray.org.x, ray.org.y, ray.org.z);
    direction = moab::CartVect(ray.dir.x, ray.dir.y, ray.dir.z);
    double dist;
    double huge_val = 1E37;
    double* nonneg_ray_len = &huge_val;

    bool hit = moab::GeomUtil::plucker_ray_tri_intersect(coords,
							 origin,
							 direction,
							 dist,
							 nonneg_ray_len);

#ifdef VERBOSE_MODE
    std::cout << "Triangle Vert Coords: " << coords[0] << coords[1] << coords[2] << std::endl;
    if (hit) std::cout << "Hit found at distance: " << dist << std::endl;
    std::cout << std::endl;
#endif
    
    if (hit && dist < ray.tfar) {

      moab::CartVect normal = (coords[2]-coords[0]) * (coords[1]-coords[0]);

      ray.primID = eh;
      ray.tfar = dist;
      ray.geomID = tray.setID;
      ray.Ng = tray.sense? (normal * -1.0).array() : normal.array();
    }

    return hit;
  }

};

typedef MBTriangleRefT<moab::EntityHandle> MBTriangleRef;

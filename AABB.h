
#include <cstring>
#include <algorithm>


struct AABB {

  float extents[6]; // extents of box {x_min, y_min, z_min, x_max, y_max, z_max}

  inline AABB() {}

  inline AABB( float x_min, float y_min, float z_min, float x_max, float y_max, float z_max) { extents[0] = x_min;
                                                                                               extents[1] = y_min;
											       extents[2] = z_min;
											       extents[3] = x_max;
											       extents[4] = y_max;
											       extents[5] = z_max; }
  
  inline AABB( const float ext[6] ) { std::memcpy(extents,  ext, sizeof(extents)); }
  
  inline AABB& operator =( const AABB& other) { std::memcpy(extents, other.extents, sizeof(extents)); return *this; }

  inline AABB( const AABB& other) { std::memcpy(extents, other.extents, sizeof(extents)); }

  inline void update( const float x_val, const float y_val, const float z_val ) { extents[0] = std::min(extents[0], x_val);
                                                                                  extents[1] = std::min(extents[1], y_val);
										  extents[2] = std::min(extents[2], z_val);
										  extents[3] = std::max(extents[3], x_val);
										  extents[4] = std::max(extents[4], y_val);
										  extents[5] = std::max(extents[5], z_val); }
  
  inline void update( const float xyz[3] ) { extents[0] = std::min(extents[0], xyz[0]);
                                             extents[1] = std::min(extents[1], xyz[1]);
					     extents[2] = std::min(extents[2], xyz[2]);
					     extents[3] = std::max(extents[3], xyz[0]);
					     extents[4] = std::max(extents[4], xyz[1]);
					     extents[5] = std::max(extents[5], xyz[2]); }

  
};

  

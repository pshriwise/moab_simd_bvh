#include <assert.h>
#include <iostream>
#include <math.h>

struct Vec3f {

    float x,y,z;
 

    inline Vec3f() {}
    
    inline Vec3f( const Vec3f &other) { x = other.x;
                                        y = other.y;
					z = other.z; }
      
    inline Vec3f& operator =( const Vec3f& other ) { x = other.x;
						     y = other.y;
						     z = other.z;
						     return *this; }

    inline const float& operator []( const size_t axis) const { assert(axis < 3); return (&x)[axis]; }
    inline       float& operator []( const size_t axis)       { assert(axis < 3); return (&x)[axis]; }
    
      
    inline Vec3f( const float x, const float y, const float z) : x(x), y(y), z(z) {}

    inline Vec3f( const float v[3] ) : x(v[0]), y(v[1]), z(v[2]) {}

    inline float length() const { return sqrtf(x*x + y*y + z*z); }



    friend std::ostream& operator <<(std::ostream &os, Vec3f const&v);

  };

inline std::ostream& operator <<(std::ostream &os, Vec3f const&v) {
  return os << '[' << v[0] << ' ' << v[1] << ' ' << v[2] << ']';
}

inline void ge_mask( const Vec3f &a, const Vec3f &b, bool mask[3] ) { mask[0] = a.x >= b.x;
                                                                      mask[1] = a.y >= b.y;
								      mask[2] = a.z >= b.z; }

inline void le_mask( const Vec3f &a, const Vec3f &b, bool mask[3] ) { mask[0] = a.x <= b.x;
                                                                      mask[1] = a.y <= b.y;
								      mask[2] = a.z <= b.z; }

inline bool all(bool b[3]) { return b[0] && b[1] && b[2]; }

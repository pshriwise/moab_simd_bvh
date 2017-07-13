

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
      
    inline Vec3f( const float x, const float y, const float z) : x(x), y(y), z(z) {}
    
  };

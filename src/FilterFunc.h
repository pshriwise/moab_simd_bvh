
#include "Ray.h"

template<typename V, typename P, typename I>
struct Filter {
  typedef void(*FilterFunc)(RayT<V,P,I> &ray, void* mesh_ptr);
};


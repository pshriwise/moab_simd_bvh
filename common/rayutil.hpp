
#include<math.h>

static const double PI = acos(-1.0);
static const double denom = 1.0 / ((double) RAND_MAX);
static const double denomPI = PI * denom;

inline void RNDVEC(moab::CartVect& uvw, const double& az = 0.0) {
  // denom normalizes rand values (see global defines)
  double theta = az * denom * rand(); // randomly samples from 0 to az. (Default az is 2PI)
  double u = 2 * denom * rand() - 1; // randomly samples from -1 to 1.
  uvw[0] = sqrt(1 - u * u) * cos(theta);
  uvw[1] = sqrt(1 - u * u) * sin(theta);
  uvw[2] = u;

}

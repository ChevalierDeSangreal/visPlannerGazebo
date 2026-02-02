// utils.hpp
#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <cmath>

// Wrap angle to [-pi, pi]
static float wrap_pi(float x)
{
  while (x >  M_PI) x -= 2.f * M_PI;
  while (x < -M_PI) x += 2.f * M_PI;
  return x;
}

#endif  // UTILS_HPP_


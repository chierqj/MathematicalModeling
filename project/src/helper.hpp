#ifndef HELPER_HPP
#define HELPER_HPP
#include <random>

std::default_random_engine e(time(nullptr));
int randomInt(int l, int r) {
  std::uniform_int_distribution<int> u(l, r);
  return u(e);
}
double randomDouble(double l, double r) {
  std::uniform_real_distribution<double> u(l, r);
  return u(e);
}

#endif
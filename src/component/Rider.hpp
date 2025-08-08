#pragma once
#include <cmath>

struct RiderComponent {
  u_int16_t weight;
  u_int16_t ftp;

  RiderComponent(u_int16_t weight, u_int16_t ftp) : weight(weight), ftp(ftp) {}
};


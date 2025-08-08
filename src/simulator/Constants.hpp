#pragma once

namespace Physics {

float g = 9.8067;
float Cd = 0.63;     // drag coefficient -> todo move to rider
float A = 0.509;     // frontal area -> todo move to rider
float rho = 1.22601; // density of air -> todo move to segment


float getFgravity(float sinTheta, uint16_t riderWeight) {
  return Physics::g * sinTheta * riderWeight;
};

float getFrolling(float cosTheta, float roadQuality, uint16_t riderWeight) {
  return Physics::g * cosTheta * riderWeight * roadQuality;
}

} // namespace Physics
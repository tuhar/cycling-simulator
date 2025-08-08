#pragma once
#include <ecs/Entity.hpp>

struct NextSegmentComponent {
  Entity nextSegment;

  NextSegmentComponent(Entity nextSegment) : nextSegment(nextSegment) {};
};

struct SegmentComponent {
  float roadQuality = 0.005;
  float theta;
  float windSpeed = 0;
  float cosTheta = cos(theta);
  float sinTheta = sin(theta);

  SegmentComponent(float grade) : theta(atan(grade / 100)) {};
};
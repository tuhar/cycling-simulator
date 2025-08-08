#pragma once

#include <ecs/Entity.hpp>

struct DistanceComponent {
    Entity segment;
  double distanceRemainingOnSegment;
  double distanceRemainingTotal;
  double coveredDistance;
  double fGravity;
  double fRolling;

  DistanceComponent(Entity segment, double segmentLength, double totalLength, double fGravity, double fRolling)
      : segment(segment),
        distanceRemainingOnSegment(segmentLength),
        distanceRemainingTotal(totalLength),
        coveredDistance(0.0),
        fGravity(fGravity),
        fRolling(fRolling) {};
};
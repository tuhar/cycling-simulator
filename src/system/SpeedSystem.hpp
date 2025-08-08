#pragma once

#include <component/Distance.hpp>
#include <component/Energy.hpp>
#include <component/Position.hpp>
#include <component/Rider.hpp>
#include <component/Segment.hpp>
#include <component/Speed.hpp>
#include <ecs/ComponentManager.hpp>
#include <simulator/Constants.hpp>
#include <simulator/Level.hpp>


float getPowerOutput(RiderComponent* rider, FatigueComponent* energyConsumedComponent, float completition) {
  if (completition > 90) return energyConsumedComponent->maxPower;       // Start strong
  if (completition > 10) return rider->ftp * 0.50;  // Steady pace
  return energyConsumedComponent->maxPower;         // Sprint finish
}

float getFdrag(SegmentComponent* segment, SpeedComponent* speedComponent) {
  return 0.5 * Physics::Cd * Physics::A * Physics::rho * pow((speedComponent->speed + segment->windSpeed), 2.0);
}

void updateSpeed(ComponentManager& cm, float dt, Level& level) {
  auto view = cm.view<RiderComponent, DistanceComponent, SpeedComponent, FatigueComponent, PositionComponent>();
  auto& segmentStorage = cm.get<SegmentComponent>();
  auto* riders = view.getComponent<RiderComponent>().data();
  auto* distances = view.getComponent<DistanceComponent>().data();
  auto* speeds = view.getComponent<SpeedComponent>().data();
  auto* fatigues = view.getComponent<FatigueComponent>().data();
  auto* positions = view.getComponent<PositionComponent>().data();

  std::unordered_map<Entity, SegmentComponent*> segmentCache;

  for (size_t i = 0; i < view.denseArray.size(); i++) {
    RiderComponent* rider = riders[i];
    DistanceComponent* distance = distances[i];
    auto segIt = segmentCache.find(distance->segment);
    if (segIt == segmentCache.end()) {
      segIt = segmentCache.emplace(distance->segment, segmentStorage.get(distance->segment)).first;
    }
    SegmentComponent* segment = segIt->second;
    SpeedComponent* speed = speeds[i];
    FatigueComponent* fatigue = fatigues[i];
    PositionComponent* position = positions[i];

    float completition =
        (distance->distanceRemainingTotal / level.routeLenght) *
        100;  // todo fix inverted distance
    float powerOutput = getPowerOutput(rider, fatigue, completition);
    float fResist = distance->fGravity + distance->fRolling + getFdrag(segment, speed);
    float acceleration = (powerOutput / (rider->weight * speed->speed)) - (fResist / rider->weight);
    speed->speed += dt * acceleration;
    speed->velocity.x = speed->speed * segment->cosTheta;
    speed->velocity.y = -1 * speed->speed * segment->sinTheta;
    position->position += speed->velocity * dt;

    // when more complex move to its own system
    double coveredDistance = speed->speed * dt;
    distance->distanceRemainingOnSegment -= coveredDistance;
    distance->distanceRemainingTotal -= coveredDistance;
    distance->coveredDistance = coveredDistance;

    fatigue->workDone = fResist * coveredDistance;
    fatigue->powerOutput = powerOutput;
  }
}

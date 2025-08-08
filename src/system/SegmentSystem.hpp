#pragma once

#include <component/Distance.hpp>
#include <component/Rider.hpp>
#include <component/Segment.hpp>
#include <component/Length.hpp>
#include <component/Speed.hpp>
#include <ecs/ComponentManager.hpp>
#include <simulator/Level.hpp>
#include <simulator/Constants.hpp>
#include <component/Render.hpp>
#include <component/Position.hpp>

static void updateSegment(ComponentManager& cm, Level& level) {
  auto view = cm.view<DistanceComponent, RiderComponent>();
  std::vector<Entity> garbage;
  for (size_t i = 0; i < view.denseArray.size(); i++) {
    Entity& entity = view.denseArray[i];
    DistanceComponent* distance = view.getComponent<DistanceComponent>()[i];
    if (distance->distanceRemainingTotal <= 0) {
      distance->coveredDistance = 0;
      garbage.emplace_back(entity);
      level.classification.emplace_back(entity);
      level.ridersOnRoute.erase(entity);
    } else if (distance->distanceRemainingOnSegment <= 0) {
      NextSegmentComponent& nextSegmentComponent = *cm.get<NextSegmentComponent>().get(distance->segment);
      LengthComponent& nextSegmentLength = *cm.get<LengthComponent>().get(nextSegmentComponent.nextSegment);
      SegmentComponent& segmentC = *cm.get<SegmentComponent>().get(nextSegmentComponent.nextSegment);
      RiderComponent* rider = view.getComponent<RiderComponent>()[i];
      distance->distanceRemainingOnSegment += nextSegmentLength.length;
      distance->segment = nextSegmentComponent.nextSegment;
      distance->fGravity = Physics::getFgravity(segmentC.sinTheta, rider->weight);
      distance->fRolling = Physics::getFrolling(segmentC.cosTheta, segmentC.roadQuality, rider->weight);
    }
  }
  for (Entity entity : garbage) {
    cm.remove<DistanceComponent>(entity);
    cm.remove<SpeedComponent>(entity);
    cm.remove<RenderComponent>(entity);
    cm.remove<PositionComponent>(entity);
    cm.remove<RiderComponent>(entity);
  }
}
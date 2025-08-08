#pragma once

#include <ecs/Entity.hpp>
#include <unordered_set>
#include <vector>

struct Level {
  float routeLenght = 0;
  float raceTime = 0;
  std::vector<Entity> classification;
  std::unordered_set<Entity> ridersOnRoute;
  bool finished = false;
};
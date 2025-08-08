#pragma once

#include <ecs/Entity.hpp>
#include <vector>

struct SparseSet {
  std::vector<size_t> sparseArray;
  std::vector<Entity> denseArray;

  bool contains(Entity e);
  void add(Entity e);
  void remove(Entity e);
};
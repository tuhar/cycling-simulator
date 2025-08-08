#pragma once

#include <ecs/Entity.hpp>
#include <ecs/SparseSet.hpp>
#include <vector>


template <typename Component>
struct Storage : SparseSet {
  std::vector<Component> components;

  void add(Entity e, Component&& c) {
    SparseSet::add(e);
    components.push_back(std::forward<Component>(c));
  }

  void remove(Entity e) {
    size_t lastIndex = components.size() - 1;
    std::swap(components[sparseArray[e]], components[lastIndex]);
    components.pop_back();
    SparseSet::remove(e);
  }

  Component* get(Entity e) {
    // todo safety checks
    return &components[sparseArray[e]];
  }
};
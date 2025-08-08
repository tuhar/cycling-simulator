#pragma once

#include <ecs/Entity.hpp>

struct EntityManager {
  // todo reusable entities
  Entity sequence = 0;

  Entity createEntity() {
    Entity entity = sequence++;
    return entity;
  }
};
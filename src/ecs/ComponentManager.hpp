#pragma once

#include <ecs/Entity.hpp>
#include <ecs/SparseSet.hpp>
#include <ecs/Storage.hpp>
#include <ecs/UpdateView.hpp>
#include <any>
#include <memory>
#include <typeindex>
#include <unordered_map>

struct ComponentManager {
  std::unordered_map<std::type_index, std::unique_ptr<SparseSet>> entityComponents;

  template <typename Component>
  void emplace(Entity e, Component &&component) {
    std::type_index type = typeid(std::decay_t<Component>);
    auto it = entityComponents.find(type);
    if (it == entityComponents.end()) {
      entityComponents[type] = std::make_unique<Storage<Component>>();
    }
    Storage<Component> &storage =
        static_cast<Storage<Component> &>(*entityComponents[type]);
    storage.add(e, std::forward<Component>(component));
  }
  template <typename Component>
  void remove(Entity e) {
    std::type_index type = typeid(std::decay_t<Component>);
    auto it = entityComponents.find(type);
    if (it == entityComponents.end()) {
      return;
    }
    Storage<Component> &storage =
        static_cast<Storage<Component> &>(*entityComponents[type]);
    storage.remove(e);
  }

  template <typename Component>
  Storage<Component> &get() {
    std::type_index type = typeid(std::decay_t<Component>);
    auto it = entityComponents.find(type);
    if (it == entityComponents.end()) {
      throw std::runtime_error("Unkown component " + std::string(type.name()) + "!");
    }
    return static_cast<Storage<Component> &>(*entityComponents[type]);
  }

  template <typename... Components>
  UpdateView<Components...> view() {
    UpdateView<Components...> view;
    SparseSet *smallest = nullptr;

    (([&] {
       auto &storage = get<Components>();
       if (!smallest ||
           storage.denseArray.size() < smallest->denseArray.size()) {
         smallest = &storage;
       }
     }()),
     ...);

    for (auto entity : smallest->denseArray) {
      bool hasAll = ((get<Components>().contains(entity)), ...);
      if (hasAll) {
        auto components = std::make_tuple(get<Components>().get(entity)...);
        std::apply([&](Components *... ptrs) { view.add(entity, ptrs...); }, components);
      }
    }
    return view;
  }
};
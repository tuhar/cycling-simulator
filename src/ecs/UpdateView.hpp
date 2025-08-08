#pragma once

#include <vector>

template <typename Component>
using ComponentViewVector = std::vector<Component*>;
template <typename... Components>
using ComponentViewVectorTuple = std::tuple<ComponentViewVector<Components>...>;

template <typename... Components>
struct UpdateView : SparseSet {
  ComponentViewVectorTuple<Components...> components;

  void add(Entity entity, Components*... entityComponents) {
    SparseSet::add(entity);
    addComponents(std::index_sequence_for<Components...>{},
                  entityComponents...);
  }

  template <typename TargetComponent>
  ComponentViewVector<TargetComponent>& getComponent() {
    return std::get<ComponentViewVector<TargetComponent>>(components);
  }

  void remove(Entity entity) {
    if (!SparseSet::contains(entity)) return;
    size_t lastIndex = denseArray.size() - 1;
    Entity lastEntity = denseArray[lastIndex];
    removeComponents(sparseArray[entity], sparseArray[lastEntity],
                     std::index_sequence_for<Components...>{});
    SparseSet::remove(entity);

    std::cout << "View state: " << entity << std::endl;
  }

 private:
  template <std::size_t... Is>
  void addComponents(std::index_sequence<Is...>,
                     Components*... entityComponents) {
    (..., (std::get<Is>(components).push_back(entityComponents)));
  }

  template <std::size_t... Is>
  void removeComponents(size_t index, size_t lastIndex,
                        std::index_sequence<Is...>) {
    (..., (std::swap(std::get<Is>(components)[index],
                     std::get<Is>(components)[lastIndex]),
           std::get<Is>(components).pop_back()));
  }
};
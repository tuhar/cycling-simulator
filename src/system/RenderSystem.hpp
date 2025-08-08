#pragma once

#include <fmt/format.h>

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <component/Distance.hpp>
#include <component/Energy.hpp>
#include <component/Name.hpp>
#include <component/Position.hpp>
#include <component/Render.hpp>
#include <component/Speed.hpp>
#include <ecs/ComponentManager.hpp>

void render(sf::RenderWindow& window, ComponentManager& cm, sf::View& camera, sf::View& hud) {
  auto view = cm.view<RenderComponent, SpeedComponent, PositionComponent>();
  auto& nameStorage = cm.get<NameComponent>();
  auto& fatigueStorage = cm.get<FatigueComponent>();
  auto& distanceStorage = cm.get<DistanceComponent>();

  window.setView(hud);
  for (size_t i = 0; i < view.denseArray.size(); i++) {
    Entity& entity = view.denseArray[i];
    RenderComponent* text = view.getComponent<RenderComponent>()[i];
    SpeedComponent* speed = view.getComponent<SpeedComponent>()[i];
    NameComponent& name = *nameStorage.get(entity);
    FatigueComponent& fatigue = *fatigueStorage.get(entity);
    DistanceComponent& distance = *distanceStorage.get(entity);
    fmt::memory_buffer buf;
    fmt::format_to(std::back_inserter(buf),
                   "{} speed: {:.1f} km/h, power output: {:.3f}W, remaining: {:.1f} m, fgravity: {:.1f}, frolling: {:.1f}",
                   name.name, speed->speed * 3.6f, fatigue.powerOutput, distance.distanceRemainingTotal, distance.fGravity, distance.fRolling);
    text->text.setString(std::string(buf.data(), buf.size()));
    window.draw(text->text);
  }

  window.setView(camera);
  for (size_t i = 0; i < view.denseArray.size(); i++) {
    PositionComponent* position = view.getComponent<PositionComponent>()[i];
    RenderComponent* text = view.getComponent<RenderComponent>()[i];
    text->sprite.setPosition(position->position);
    if (camera.getCenter().x < text->sprite.getPosition().x) {
      camera.setCenter(position->position);
    }
    window.draw(text->sprite);
  }
}
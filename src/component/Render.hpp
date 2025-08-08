#pragma once

#include <SFML/Graphics.hpp>

struct RenderComponent {
  sf::Text text;
  sf::CircleShape sprite;

  RenderComponent(sf::Text text, sf::CircleShape sprite)
      : text(text), sprite(sprite) {}
};
#pragma once

#include <SFML/Graphics.hpp>

struct SpeedComponent {
  float speed = 0.1;
  sf::Vector2f velocity = {0, 0};
};
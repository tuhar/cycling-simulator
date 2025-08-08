#pragma once

#include <SFML/Graphics.hpp>

struct PositionComponent {
    sf::Vector2f position;
    
    PositionComponent(sf::Vector2f position): position(position){};
};
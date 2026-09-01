#pragma once

#include <SFML/Graphics.hpp>

struct CubicBezier {
    sf::Vector2f p0;
    sf::Vector2f p1;
    sf::Vector2f p2;
    sf::Vector2f p3;

    sf::Vector2f Evaluate(float t) const;
};

void DrawHandleLine(sf::RenderWindow& window, sf::Vector2f a, sf::Vector2f b, sf::Color color);
void DrawSolidCurve(sf::RenderWindow& window, const CubicBezier& curve, int segmentCount, sf::Color color);

#pragma once

#include <SFML/Graphics.hpp>

// A cubic Bezier curve defined by two anchor points (p0, p3) and two control points (p1, p2).
struct CubicBezier {
    sf::Vector2f p0;
    sf::Vector2f p1;
    sf::Vector2f p2;
    sf::Vector2f p3;

    sf::Vector2f Evaluate(float t) const;
};

// Draws the curve as alternating "on"/"off" segments sampled at fixed t-steps, so dashes
// visibly stretch or bunch up as the control points reshape the curve.
void DrawDashedCurve(sf::RenderWindow& window, const CubicBezier& curve, int segmentCount, float thickness, sf::Color color);

void DrawHandleLine(sf::RenderWindow& window, sf::Vector2f a, sf::Vector2f b, sf::Color color);

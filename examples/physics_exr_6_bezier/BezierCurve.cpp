#include "BezierCurve.h"
#include "MathHelpers.h"
#include <array>
#include <cmath>

sf::Vector2f CubicBezier::Evaluate(float t) const {
    float u = 1.f - t;
    float b0 = u * u * u;
    float b1 = 3.f * u * u * t;
    float b2 = 3.f * u * t * t;
    float b3 = t * t * t;
    return b0 * p0 + b1 * p1 + b2 * p2 + b3 * p3;
}

void DrawDashedCurve(sf::RenderWindow& window, const CubicBezier& curve, int segmentCount, float thickness, sf::Color color) {
    if (segmentCount < 2) segmentCount = 2;

    sf::Vector2f prevPoint = curve.Evaluate(0.f);
    for (int i = 1; i <= segmentCount; ++i) {
        float t = static_cast<float>(i) / static_cast<float>(segmentCount);
        sf::Vector2f point = curve.Evaluate(t);

        // Only draw odd-indexed segments so the curve renders as a dash/gap pattern.
        if (i % 2 == 1) {
            sf::Vector2f dir = point - prevPoint;
            float len = Math::Length(dir);
            if (len > 0.0001f) {
                sf::RectangleShape dash({ len, thickness });
                dash.setOrigin({ 0.f, thickness * 0.5f });
                dash.setPosition(prevPoint);
                dash.setRotation(sf::radians(std::atan2(dir.y, dir.x)));
                dash.setFillColor(color);
                window.draw(dash);
            }
        }

        prevPoint = point;
    }
}

void DrawHandleLine(sf::RenderWindow& window, sf::Vector2f a, sf::Vector2f b, sf::Color color) {
    std::array<sf::Vertex, 2> line{ sf::Vertex{a, color}, sf::Vertex{b, color} };
    window.draw(line.data(), line.size(), sf::PrimitiveType::Lines);
}

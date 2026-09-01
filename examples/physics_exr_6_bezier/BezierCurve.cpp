#include "BezierCurve.h"
#include <array>
#include <vector>

sf::Vector2f CubicBezier::Evaluate(float t) const {
    float u = 1.f - t;
    float b0 = u * u * u;
    float b1 = 3.f * u * u * t;
    float b2 = 3.f * u * t * t;
    float b3 = t * t * t;
    return b0 * p0 + b1 * p1 + b2 * p2 + b3 * p3;
}

void DrawHandleLine(sf::RenderWindow& window, sf::Vector2f a, sf::Vector2f b, sf::Color color) {
    std::array<sf::Vertex, 2> line{ sf::Vertex{a, color}, sf::Vertex{b, color} };
    window.draw(line.data(), line.size(), sf::PrimitiveType::Lines);
}

void DrawSolidCurve(sf::RenderWindow& window, const CubicBezier& curve, int segmentCount, sf::Color color) {
    if (segmentCount < 1) segmentCount = 1;

    std::vector<sf::Vertex> vertices;
    vertices.reserve(segmentCount + 1);

    for (int i = 0; i <= segmentCount; ++i) {
        float t = static_cast<float>(i) / static_cast<float>(segmentCount);
        vertices.push_back(sf::Vertex{ curve.Evaluate(t), color });
    }

    window.draw(vertices.data(), vertices.size(), sf::PrimitiveType::LineStrip);
}

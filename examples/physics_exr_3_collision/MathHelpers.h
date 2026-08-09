#pragma once

#include <SFML/Graphics.hpp>
#include <cmath>

namespace Math {
const float PI = 3.14159265359f;

inline float Dot(const sf::Vector2f& a, const sf::Vector2f& b) {
    return a.x * b.x + a.y * b.y;
}

inline float Cross(const sf::Vector2f& a, const sf::Vector2f& b) {
    return a.x * b.y - a.y * b.x;
}

inline float LengthSq(const sf::Vector2f& v) {
    return v.x * v.x + v.y * v.y;
}

inline float Length(const sf::Vector2f& v) {
    return std::sqrt(LengthSq(v));
}

inline sf::Vector2f Normalize(const sf::Vector2f& v) {
    float len = Length(v);
    if (len > 0.0001f) {
        return v / len;
    }
    return sf::Vector2f(0.f, 0.f);
}

inline float Distance(const sf::Vector2f& a, const sf::Vector2f& b) {
    return Length(a - b);
}

inline sf::Vector2f Rotate(const sf::Vector2f& v, float angleRad) {
    float cosA = std::cos(angleRad);
    float sinA = std::sin(angleRad);
    return sf::Vector2f(v.x * cosA - v.y * sinA, v.x * sinA + v.y * cosA);
}
}

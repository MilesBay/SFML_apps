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

inline float AngleOf(const sf::Vector2f& v) {
    return std::atan2(v.y, v.x);
}

inline float Lerp(float a, float b, float t) {
    return a + (b - a) * t;
}

inline sf::Vector2f Lerp(const sf::Vector2f& a, const sf::Vector2f& b, float t) {
    return a + (b - a) * t;
}

inline sf::Color LerpColor(const sf::Color& a, const sf::Color& b, float t) {
    return sf::Color(
        static_cast<std::uint8_t>(Lerp(static_cast<float>(a.r), static_cast<float>(b.r), t)),
        static_cast<std::uint8_t>(Lerp(static_cast<float>(a.g), static_cast<float>(b.g), t)),
        static_cast<std::uint8_t>(Lerp(static_cast<float>(a.b), static_cast<float>(b.b), t)),
        static_cast<std::uint8_t>(Lerp(static_cast<float>(a.a), static_cast<float>(b.a), t)));
}
}

#include "Physics.h"
#include <algorithm>

float AngleBetween(const sf::Vector2f& v1, const sf::Vector2f& v2) {
    float len1 = Math::Length(v1);
    float len2 = Math::Length(v2);
    if (len1 < 0.001f || len2 < 0.001f) return 0.f;

    float ratio = Math::Dot(v1, v2) / (len1 * len2);
    ratio = std::max(-1.f, std::min(1.f, ratio));
    return std::acos(ratio);
}

bool IsPointInTriangle(const sf::Vector2f& P, const sf::Vector2f& A, const sf::Vector2f& B, const sf::Vector2f& C) {
    sf::Vector2f v1 = A - P;
    sf::Vector2f v2 = B - P;
    sf::Vector2f v3 = C - P;

    float angle1 = AngleBetween(v1, v2);
    float angle2 = AngleBetween(v2, v3);
    float angle3 = AngleBetween(v3, v1);

    float sum = angle1 + angle2 + angle3;
    return std::abs(sum - 2.f * Math::PI) < 0.05f;
}
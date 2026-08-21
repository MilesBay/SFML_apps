#include "Kinematics.h"

KinematicChain::KinematicChain(const std::vector<float>& segmentLengths, sf::Vector2f origin, sf::Vector2f initialDir)
    : lengths(segmentLengths) {
    joints.resize(lengths.size() + 1);
    joints[0] = origin;

    sf::Vector2f dir = Math::Normalize(initialDir);
    for (size_t i = 0; i < lengths.size(); ++i) {
        joints[i + 1] = joints[i] + dir * lengths[i];
    }
}

void SolveForwardKinematics(KinematicChain& chain, sf::Vector2f rootPos, const std::vector<float>& localAngles) {
    chain.joints[0] = rootPos;

    float cumulativeAngle = 0.f;
    for (size_t i = 0; i < chain.lengths.size(); ++i) {
        cumulativeAngle += localAngles[i];
        sf::Vector2f dir = Math::Rotate(sf::Vector2f(0.f, -1.f), cumulativeAngle);
        chain.joints[i + 1] = chain.joints[i] + dir * chain.lengths[i];
    }
}

void DrawChain(sf::RenderWindow& window, const KinematicChain& chain,
    float thicknessStart, float thicknessEnd,
    const sf::Color& colorStart, const sf::Color& colorEnd) {
    size_t segmentCount = chain.SegmentCount();
    if (segmentCount == 0) return;

    for (size_t i = 0; i < segmentCount; ++i) {
        float t = segmentCount > 1 ? static_cast<float>(i) / static_cast<float>(segmentCount - 1) : 0.f;
        float thickness = Math::Lerp(thicknessStart, thicknessEnd, t);
        sf::Color color = Math::LerpColor(colorStart, colorEnd, t);

        const sf::Vector2f& a = chain.joints[i];
        const sf::Vector2f& b = chain.joints[i + 1];
        sf::Vector2f delta = b - a;
        float length = Math::Length(delta);
        if (length < 0.0001f) continue;

        sf::RectangleShape rect(sf::Vector2f(length, thickness));
        rect.setOrigin({ 0.f, thickness * 0.5f });
        rect.setPosition(a);
        rect.setRotation(sf::radians(std::atan2(delta.y, delta.x)));
        rect.setFillColor(color);
        window.draw(rect);
    }
}

void DrawJoint(sf::RenderWindow& window, const sf::Vector2f& pos, float radius, sf::Color color) {
    sf::CircleShape shape(radius);
    shape.setOrigin({ radius, radius });
    shape.setPosition(pos);
    shape.setFillColor(color);
    window.draw(shape);
}

#include "Kinematics.h"

namespace {
sf::Vector2f SafeDirection(const sf::Vector2f& from, const sf::Vector2f& to, const sf::Vector2f& fallback) {
    sf::Vector2f delta = to - from;
    if (Math::LengthSq(delta) < 0.0001f) {
        return fallback;
    }
    return Math::Normalize(delta);
}
}

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

void SolveFABRIK(KinematicChain& chain, sf::Vector2f basePos, sf::Vector2f target, int iterations) {
    std::vector<sf::Vector2f>& joints = chain.joints;
    const std::vector<float>& lengths = chain.lengths;
    const size_t n = joints.size();
    if (n < 2) return;

    float totalLength = 0.f;
    for (float len : lengths) totalLength += len;

    if (Math::Distance(basePos, target) >= totalLength) {
        sf::Vector2f dir = SafeDirection(basePos, target, sf::Vector2f(0.f, -1.f));
        joints[0] = basePos;
        for (size_t i = 0; i < lengths.size(); ++i) {
            joints[i + 1] = joints[i] + dir * lengths[i];
        }
        return;
    }

    const float tolerance = 0.5f;
    for (int iter = 0; iter < iterations; ++iter) {
        if (Math::Distance(joints.back(), target) < tolerance) break;

        // Backward pass
        joints[n - 1] = target;
        for (size_t i = n - 2; i < n; --i) {
            sf::Vector2f dir = SafeDirection(joints[i + 1], joints[i], sf::Vector2f(0.f, -1.f));
            joints[i] = joints[i + 1] + dir * lengths[i];
        }

        // Forward pass
        joints[0] = basePos;
        for (size_t i = 1; i < n; ++i) {
            sf::Vector2f dir = SafeDirection(joints[i - 1], joints[i], sf::Vector2f(0.f, 1.f));
            joints[i] = joints[i - 1] + dir * lengths[i - 1];
        }
    }
}

void SolveHeadFollowIK(KinematicChain& chain, sf::Vector2f headTarget) {
    std::vector<sf::Vector2f>& joints = chain.joints;
    const std::vector<float>& lengths = chain.lengths;

    joints[0] = headTarget;
    for (size_t i = 1; i < joints.size(); ++i) {
        sf::Vector2f dir = SafeDirection(joints[i - 1], joints[i], sf::Vector2f(0.f, 1.f));
        joints[i] = joints[i - 1] + dir * lengths[i - 1];
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

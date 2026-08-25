#pragma once

#include "MathHelpers.h"
#include <SFML/Graphics.hpp>
#include <vector>

struct KinematicChain {
    std::vector<sf::Vector2f> joints;
    std::vector<float> lengths;

    KinematicChain(const std::vector<float>& segmentLengths, sf::Vector2f origin, sf::Vector2f initialDir = sf::Vector2f(0.f, -1.f));
    size_t SegmentCount() const { return lengths.size(); }
};

// --- Forward Kinematics ---
void SolveForwardKinematics(KinematicChain& chain, sf::Vector2f rootPos, const std::vector<float>& localAngles);

// --
void SolveFABRIK(KinematicChain& chain, sf::Vector2f basePos, sf::Vector2f target, int iterations = 10);

// --- Inverse Kinematics: free-tailed follow chain ---
void SolveHeadFollowIK(KinematicChain& chain, sf::Vector2f headTarget);

// --- Drawing ---
void DrawChain(sf::RenderWindow& window, const KinematicChain& chain,
    float thicknessStart, float thicknessEnd,
    const sf::Color& colorStart, const sf::Color& colorEnd);

void DrawJoint(sf::RenderWindow& window, const sf::Vector2f& pos, float radius, sf::Color color);

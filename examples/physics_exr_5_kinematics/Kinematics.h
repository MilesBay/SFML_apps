#pragma once

#include "MathHelpers.h"
#include <SFML/Graphics.hpp>
#include <vector>

// A kinematic chain: a sequence of rigid, fixed-length segments connected end
// to end by joints. Solvers below move the joints around but never change
// `lengths`, so no segment can stretch or shrink.
struct KinematicChain {
    std::vector<sf::Vector2f> joints;  // size = segmentCount + 1
    std::vector<float> lengths;        // size = segmentCount

    // Lays the chain out as a straight line of `segmentLengths.size()` segments,
    // starting at `origin` and running along `initialDir`.
    KinematicChain(const std::vector<float>& segmentLengths, sf::Vector2f origin, sf::Vector2f initialDir = sf::Vector2f(0.f, -1.f));

    size_t SegmentCount() const { return lengths.size(); }
};

// --- Forward Kinematics ---
// Resolves every joint from the root outward. `localAngles[i]` is segment i's
// angle relative to segment i-1 (angle 0 = straight up); the accumulated sum
// of local angles up to i gives segment i's world angle, so rotating a lower
// segment carries every segment above it along with it.
void SolveForwardKinematics(KinematicChain& chain, sf::Vector2f rootPos, const std::vector<float>& localAngles);

// --- Inverse Kinematics: FABRIK (anchored root) ---
// Reaches the chain's end effector toward `target` while `chain.joints[0]`
// stays pinned to `basePos`. If the target is farther than the chain's total
// length it is approached by fully extending the chain in a straight line.
void SolveFABRIK(KinematicChain& chain, sf::Vector2f basePos, sf::Vector2f target, int iterations = 10);

// --- Inverse Kinematics: free-tailed follow chain ---
// Places the head (`chain.joints[0]`) directly at `headTarget`, then
// re-projects every following joint to sit exactly `lengths[i-1]` from the
// one before it, so the body trails behind the head without a fixed end.
void SolveHeadFollowIK(KinematicChain& chain, sf::Vector2f headTarget);

// --- Drawing ---
// Draws the chain as tapered segments, lerping thickness and colour linearly
// from the root (index 0) to the tip (index SegmentCount()-1).
void DrawChain(sf::RenderWindow& window, const KinematicChain& chain,
    float thicknessStart, float thicknessEnd,
    const sf::Color& colorStart, const sf::Color& colorEnd);

void DrawJoint(sf::RenderWindow& window, const sf::Vector2f& pos, float radius, sf::Color color);

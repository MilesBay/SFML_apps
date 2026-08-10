#include "Physics.h"
#include <iostream>
#include <vector>

int main() {
    std::cout << "--- Commit 2 Smoke Test: Capsule & Convex Body Resolution ---\n";

    // 1. Capsule overlap test
    Capsule capA(sf::Vector2f(100.f, 100.f), 60.f, 15.f, 0.f, 1.f, 0.5f, sf::Color::Red);
    Capsule capB(sf::Vector2f(110.f, 100.f), 60.f, 15.f, 0.f, 1.f, 0.5f, sf::Color::Blue);
    std::cout << "Initial CapA X: " << capA.position.x << " | CapB X: " << capB.position.x << "\n";
    ResolveCapsuleCollision(capA, capB);
    std::cout << "Resolved CapA X: " << capA.position.x << " | CapB X: " << capB.position.x << "\n";

    // 2. Convex Polygon SAT test
    std::vector<sf::Vector2f> boxVerts = { {-10.f, -10.f}, {10.f, -10.f}, {10.f, 10.f}, {-10.f, 10.f} };
    ConvexBody polyA(sf::Vector2f(200.f, 200.f), boxVerts, 0.f, 1.f, 0.5f, sf::Color::Green);
    ConvexBody polyB(sf::Vector2f(210.f, 200.f), boxVerts, 0.f, 1.f, 0.5f, sf::Color::Yellow);

    SATResult sat = CheckSAT(polyA.GetWorldVertices(), polyA.position, polyB.GetWorldVertices(), polyB.position);
    std::cout << "Convex Collision Detected: " << (sat.collided ? "YES" : "NO")
              << " | Overlap Depth: " << sat.depth << "\n";

    ResolveConvexCollision(polyA, polyB);
    std::cout << "Resolved PolyA X: " << polyA.position.x << " | PolyB X: " << polyB.position.x << "\n";

    return 0;
}
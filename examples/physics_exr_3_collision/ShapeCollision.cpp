// ShapeCollision.cpp
#include "Physics.h"
#include <iostream>

int main() {
    std::cout << "--- Commit 1 Smoke Test: CircleBody & Euler Integration ---\n";

    CircleBody circle(sf::Vector2f(100.f, 100.f), 20.f, 2.0f, 0.7f, sf::Color::Green);
    const sf::Vector2f gravity(0.f, 9.8f);

    std::cout << "Initial pos: (" << circle.position.x << ", " << circle.position.y << ")\n";

    // Apply a lateral impulse and simulate 1.0s with gravity
    circle.ApplyForce(sf::Vector2f(50.f, 0.f));
    circle.UpdatePhysics(1.0f, gravity);

    std::cout << "Pos after 1s: (" << circle.position.x << ", " << circle.position.y << ")\n";
    std::cout << "Velocity: (" << circle.velocity.x << ", " << circle.velocity.y << ")\n";

    return 0;
}
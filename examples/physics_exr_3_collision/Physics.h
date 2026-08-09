#pragma once

#include "MathHelpers.h"
#include <SFML/Graphics.hpp>
#include <vector>

// Base Physics Body
class PhysicsBody {
public:
    sf::Vector2f position;
    sf::Vector2f velocity;
    sf::Vector2f force;
    float mass;
    float restitution;

    PhysicsBody(sf::Vector2f pos, float m, float rest)
        : position(pos), velocity(0.f, 0.f), force(0.f, 0.f), mass(m), restitution(rest) {
        if (mass < 0.001f) mass = 0.001f;
    }

    virtual ~PhysicsBody() {}

    void ApplyForce(const sf::Vector2f& f) {
        force += f;
    }

    void UpdatePhysics(float dt, const sf::Vector2f& gravity) {
        if (mass <= 0.f) return;

        sf::Vector2f acceleration = (force / mass) + gravity;
        velocity += acceleration * dt;
        position += velocity * dt;

        force = sf::Vector2f(0.f, 0.f);
    }
};

class CircleBody : public PhysicsBody {
public:
    float radius;
    sf::Color color;

    CircleBody(sf::Vector2f pos, float r, float m, float rest, sf::Color col)
        : PhysicsBody(pos, m, rest), radius(r), color(col) {
    }
};

// Geometric Utility Functions
float AngleBetween(const sf::Vector2f& v1, const sf::Vector2f& v2);
bool IsPointInTriangle(const sf::Vector2f& P, const sf::Vector2f& A, const sf::Vector2f& B, const sf::Vector2f& C);
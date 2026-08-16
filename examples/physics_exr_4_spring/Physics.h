#pragma once

#include "MathHelpers.h"
#include <SFML/Graphics.hpp>
#include <vector>

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

class Particle : public PhysicsBody {
public:
    Particle(sf::Vector2f pos, float m, float rest)
        : PhysicsBody(pos, m, rest) {}
};

void ResolveEdgeCollision(Particle& particle, float radius, float screenWidth, float screenHeight);
void DrawParticle(sf::RenderWindow& window, const Particle& particle, float radius, sf::Color color);

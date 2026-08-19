#pragma once

#include "MathHelpers.h"
#include <SFML/Graphics.hpp>
#include <vector>

// Base Physics Body: symplectic Euler integration (v += a*dt, then x += v*dt)
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

// A single node of the rope chain
class Particle : public PhysicsBody {
public:
    Particle(sf::Vector2f pos, float m, float rest)
        : PhysicsBody(pos, m, rest) {
    }
};

// A damped spring connecting two particles (by index into the chain)
struct Spring {
    size_t indexA;
    size_t indexB;
    float restLength;
    float stiffness;
    float damping;
};

// Function Declarations
void ApplySpringForce(std::vector<Particle>& particles, const Spring& spring);
void ResolveEdgeCollision(Particle& particle, float radius, float screenWidth, float screenHeight);

float MassToRadius(float mass, float minMass, float maxMass);
float MassToThickness(float mass, float minMass, float maxMass);

void DrawParticle(sf::RenderWindow& window, const Particle& particle, float radius, sf::Color color);
void DrawRopeSegment(sf::RenderWindow& window, const sf::Vector2f& a, const sf::Vector2f& b, float thickness, sf::Color color);

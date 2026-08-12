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

// Derived Shapes
class Capsule : public PhysicsBody {
public:
    float length;
    float radius;
    float angle;
    float angularSpeed;
    sf::Color color;

    Capsule(sf::Vector2f pos, float len, float rad, float rot, float m, float rest, sf::Color col)
        : PhysicsBody(pos, m, rest), length(len), radius(rad), angle(rot), angularSpeed(0.2f), color(col) {
    }

    sf::Vector2f GetWorldA() const;
    sf::Vector2f GetWorldB() const;
    void UpdateRotation(float dt);
};

class ConvexBody : public PhysicsBody {
public:
    std::vector<sf::Vector2f> localVertices;
    float angle;
    float angularSpeed;
    sf::Color color;

    ConvexBody(sf::Vector2f pos, const std::vector<sf::Vector2f>& verts, float rot, float m, float rest, sf::Color col)
        : PhysicsBody(pos, m, rest), localVertices(verts), angle(rot), angularSpeed(0.25f), color(col) {
    }

    std::vector<sf::Vector2f> GetWorldVertices() const;
    void UpdateRotation(float dt);
};

class CircleBody : public PhysicsBody {
public:
    float radius;
    sf::Color color;

    CircleBody(sf::Vector2f pos, float r, float m, float rest, sf::Color col)
        : PhysicsBody(pos, m, rest), radius(r), color(col) {
    }
};

struct SATResult {
    bool collided;
    sf::Vector2f normal;
    float depth;
};

// Function Declarations
sf::Vector2f ClosestPointOnSegment(const sf::Vector2f& X, const sf::Vector2f& Y, const sf::Vector2f& P);
void ResolveCapsuleCollision(Capsule& capA, Capsule& capB);
SATResult CheckSAT(const std::vector<sf::Vector2f>& vertsA, const sf::Vector2f& centerA,
    const std::vector<sf::Vector2f>& vertsB, const sf::Vector2f& centerB);
void ResolveConvexCollision(ConvexBody& bodyA, ConvexBody& bodyB);
float AngleBetween(const sf::Vector2f& v1, const sf::Vector2f& v2);
bool IsPointInTriangle(const sf::Vector2f& P, const sf::Vector2f& A, const sf::Vector2f& B, const sf::Vector2f& C);

void DrawCapsule(sf::RenderWindow& window, const Capsule& cap);
void DrawConvex(sf::RenderWindow& window, const ConvexBody& body);

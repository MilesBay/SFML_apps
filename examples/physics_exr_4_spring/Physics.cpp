#include "Physics.h"
#include <algorithm>
#include <cmath>

void ApplySpringForce(std::vector<Particle>& particles, const Spring& spring) {
    Particle& a = particles[spring.indexA];
    Particle& b = particles[spring.indexB];

    sf::Vector2f delta = b.position - a.position;
    float dist = Math::Length(delta);
    if (dist < 0.0001f) return;

    sf::Vector2f dir = delta / dist;
    float displacement = dist - spring.restLength;

    float springMag = spring.stiffness * displacement;

    sf::Vector2f relativeVelocity = b.velocity - a.velocity;
    float dampingMag = spring.damping * Math::Dot(relativeVelocity, dir);

    sf::Vector2f force = dir * (springMag + dampingMag);

    a.ApplyForce(force);
    b.ApplyForce(-force);
}

void ResolveEdgeCollision(Particle& particle, float radius, float screenWidth, float screenHeight) {
    if (particle.position.x - radius < 0.f) {
        particle.position.x = radius;
        particle.velocity.x = std::abs(particle.velocity.x) * particle.restitution;
    } else if (particle.position.x + radius > screenWidth) {
        particle.position.x = screenWidth - radius;
        particle.velocity.x = -std::abs(particle.velocity.x) * particle.restitution;
    }

    if (particle.position.y - radius < 0.f) {
        particle.position.y = radius;
        particle.velocity.y = std::abs(particle.velocity.y) * particle.restitution;
    } else if (particle.position.y + radius > screenHeight) {
        particle.position.y = screenHeight - radius;
        particle.velocity.y = -std::abs(particle.velocity.y) * particle.restitution;
    }
}

float MassToRadius(float mass, float minMass, float maxMass) {
    float t = (mass - minMass) / (maxMass - minMass);
    t = std::max(0.f, std::min(1.f, t));
    return 5.f + t * (14.f - 5.f);
}

float MassToThickness(float mass, float minMass, float maxMass) {
    float t = (mass - minMass) / (maxMass - minMass);
    t = std::max(0.f, std::min(1.f, t));
    return 4.f + t * (22.f - 4.f);
}

void DrawParticle(sf::RenderWindow& window, const Particle& particle, float radius, sf::Color color) {
    sf::CircleShape shape(radius);
    shape.setOrigin({ radius, radius });
    shape.setPosition(particle.position);
    shape.setFillColor(color);
    window.draw(shape);
}

void DrawRopeSegment(sf::RenderWindow& window, const sf::Vector2f& a, const sf::Vector2f& b, float thickness, sf::Color color) {
    sf::Vector2f delta = b - a;
    float length = Math::Length(delta);
    if (length < 0.0001f) return;

    sf::RectangleShape rect(sf::Vector2f(length, thickness));
    rect.setOrigin({ 0.f, thickness * 0.5f });
    rect.setPosition(a);
    rect.setRotation(sf::radians(std::atan2(delta.y, delta.x)));
    rect.setFillColor(color);
    window.draw(rect);
}

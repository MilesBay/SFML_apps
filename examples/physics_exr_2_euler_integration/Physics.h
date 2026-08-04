#pragma once

#include <SFML/Graphics.hpp>
#include <cmath>

namespace PhysicsLibrary
{
constexpr float PixelsPerMeter       = 30.f;
constexpr float G_Gravity            = 9.8f;
constexpr float G_MassSizeMultiplier = 8.f;
constexpr float G_JumpImpulseStrength = 6.f;
constexpr float G_Restitution        = 0.65f;
constexpr float G_MassGrowthRate     = 15.f;
} // namespace PhysicsLibrary

class PhysicsObject
{
public:
    PhysicsObject(float mass, sf::Vector2f position);

    void applyForce(sf::Vector2f force);
    void applyImpulse(sf::Vector2f impulse);
    void updatePhysics(float dt);
    void collideObject(const sf::RenderWindow& window);

    float        getMass() const;
    void         setMass(float mass);
    float        getRadius() const;

    sf::Vector2f getPosition() const;
    void         setPosition(sf::Vector2f position);
    sf::Vector2f getVelocity() const;
    void         setVelocity(sf::Vector2f velocity);

private:
    void updateRadius();

    float        m_mass = 1.f;
    sf::Vector2f m_velocity;
    sf::Vector2f m_position;
    sf::Vector2f m_acceleration;
    float        m_radius = 0.f;
};

#pragma once

#include <SFML/Graphics.hpp>

namespace PhysicsLibrary
{
constexpr float PixelsPerMeter = 30.f;
constexpr float G_Gravity      = 9.8f;
constexpr float G_Restitution  = 0.65f;
} // namespace PhysicsLibrary

class PhysicsObject
{
public:
    PhysicsObject(float mass, sf::Vector2f position);

    void applyForce(sf::Vector2f force);
    void updatePhysics(float dt);
    void collideObject(const sf::RenderWindow& window);

    float        getMass() const;
    float        getRadius() const;
    sf::Vector2f getPosition() const;
    sf::Vector2f getVelocity() const;

private:
    float        m_mass = 1.f;
    sf::Vector2f m_velocity;
    sf::Vector2f m_position;
    sf::Vector2f m_acceleration;
    float        m_radius = 15.f;
};

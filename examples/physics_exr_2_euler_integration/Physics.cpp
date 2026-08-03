#include "Physics.h"

PhysicsObject::PhysicsObject(float mass, sf::Vector2f position)
    : m_mass(mass), m_position(position)
{
}

void PhysicsObject::applyForce(sf::Vector2f force)
{
    m_acceleration += force / m_mass;
}

void PhysicsObject::updatePhysics(float dt)
{
    m_velocity += m_acceleration * dt;
    m_position += m_velocity * dt;
    m_acceleration = { 0.f, 0.f };
}

void PhysicsObject::collideObject(const sf::RenderWindow& window)
{
    const auto windowSize = static_cast<sf::Vector2f>(window.getSize());

    if (m_position.x - m_radius < 0.f)
    {
        m_position.x = m_radius;
        m_velocity.x = -m_velocity.x * PhysicsLibrary::G_Restitution;
    }
    else if (m_position.x + m_radius > windowSize.x)
    {
        m_position.x = windowSize.x - m_radius;
        m_velocity.x = -m_velocity.x * PhysicsLibrary::G_Restitution;
    }

    if (m_position.y - m_radius < 0.f)
    {
        m_position.y = m_radius;
        m_velocity.y = -m_velocity.y * PhysicsLibrary::G_Restitution;
    }
    else if (m_position.y + m_radius > windowSize.y)
    {
        m_position.y = windowSize.y - m_radius;
        m_velocity.y = -m_velocity.y * PhysicsLibrary::G_Restitution;
    }
}

float PhysicsObject::getMass() const        { return m_mass; }
float PhysicsObject::getRadius() const      { return m_radius; }
sf::Vector2f PhysicsObject::getPosition() const { return m_position; }
sf::Vector2f PhysicsObject::getVelocity() const { return m_velocity; }

#include "Physics.h"

PhysicsObject::PhysicsObject(float mass, sf::Vector2f position)
    : m_mass(mass), m_position(position)
{
    updateRadius();
}

void PhysicsObject::applyForce(sf::Vector2f force)
{
    m_acceleration += force / m_mass;
}

void PhysicsObject::applyImpulse(sf::Vector2f impulse)
{
    m_velocity += impulse / m_mass;
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

float PhysicsObject::getMass() const                  { return m_mass; }
void  PhysicsObject::setMass(float mass)              { m_mass = mass; updateRadius(); }
float PhysicsObject::getRadius() const                { return m_radius; }
sf::Vector2f PhysicsObject::getPosition() const       { return m_position; }
void PhysicsObject::setPosition(sf::Vector2f position){ m_position = position; }
sf::Vector2f PhysicsObject::getVelocity() const       { return m_velocity; }
void PhysicsObject::setVelocity(sf::Vector2f velocity){ m_velocity = velocity; }

void PhysicsObject::updateRadius()
{
    m_radius = std::sqrt(m_mass) * PhysicsLibrary::G_MassSizeMultiplier;
}

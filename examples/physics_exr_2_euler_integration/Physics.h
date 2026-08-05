#pragma once

#include <SFML/Graphics.hpp>
#include <cmath>

namespace PhysicsLibrary
{
    constexpr float PixelsPerMeter = 30.f; // treat 30px on screen as "1 metre" so forces stay screen-scaled

    constexpr float G_Gravity = 9.8f;  // m/s^2, standard gravitational acceleration
    constexpr float G_MassSizeMultiplier = 8.f;    // circle radius (px) per sqrt(mass)
    constexpr float G_WindAcceleration = 5.f;    // m/s^2, applied horizontally while W is held
    constexpr float G_JumpImpulseStrength = 6.f;    // m/s, instantaneous upward velocity change on Space
    constexpr float G_GroundFrictionCoefficient = 2.5f;   // linear friction applied while touching the floor
    constexpr float G_FluidDensity = 0.85f;   // quadratic drag strength inside the liquid rectangle
    constexpr float G_DragCoefficient = 0.47f;  // drag coefficient of a sphere
    constexpr float G_Restitution = 0.65f;  // energy kept (0-1) on a wall/floor bounce
    constexpr float G_MassGrowthRate = 15.f;   // mass gained per second while charging a placement
} // namespace PhysicsLibrary

class PhysicsObject
{
public:
    PhysicsObject(float mass, sf::Vector2f position);

    // Continuous force, e.g. gravity, wind, drag, friction (F = M * A -> A = F / M)
    void applyForce(sf::Vector2f force);

    // Instantaneous change, e.g. a jump - independent of delta time
    void applyImpulse(sf::Vector2f impulse);

    // Euler integration step, called once per frame
    void updatePhysics(float dt);

    // Keep the object inside the window, bouncing off whichever edge it crosses
    void collideObject(const sf::RenderWindow& window);

    bool isTouchingGround(float windowHeight) const;

    float getMass() const;
    void setMass(float mass);

    float getRadius() const;

    sf::Vector2f getPosition() const;
    void setPosition(sf::Vector2f position);

    sf::Vector2f getVelocity() const;
    void setVelocity(sf::Vector2f velocity);

private:
    void updateRadius();

    float        m_mass = 1.f;
    sf::Vector2f m_velocity;
    sf::Vector2f m_position;
    sf::Vector2f m_acceleration;
    float        m_radius = 0.f;
};

#include "Physics.h"

#include <SFML/Graphics.hpp>
#include <SFML/Main.hpp>
#include <iostream>
#include <vector>

namespace
{
    constexpr unsigned int windowWidth = 800;
    constexpr unsigned int windowHeight = 600;

    // Applies gravity, wind, fluid drag and ground friction to a single object for this frame
    void applyEnvironmentForces(PhysicsObject& object, const sf::FloatRect& fluidRect, bool windActive, float wHeight)
    {
        object.applyForce({ 0.f, object.getMass() * PhysicsLibrary::G_Gravity * PhysicsLibrary::PixelsPerMeter });

        if (windActive)
            object.applyForce({ object.getMass() * PhysicsLibrary::G_WindAcceleration * PhysicsLibrary::PixelsPerMeter, 0.f });

        if (fluidRect.contains(object.getPosition()))
        {
            const sf::Vector2f velocity = object.getVelocity();
            const float        speed = velocity.length();

            if (speed > 0.01f)
                object.applyForce(-velocity.normalized() * PhysicsLibrary::G_DragCoefficient * speed * speed *
                    PhysicsLibrary::G_FluidDensity);
        }

        if (object.isTouchingGround(wHeight))
        {
            const float horizontalVelocity = object.getVelocity().x;
            object.applyForce({ -horizontalVelocity * object.getMass() * PhysicsLibrary::G_GroundFrictionCoefficient, 0.f });
        }
    }

    sf::CircleShape makeCircleShape(const PhysicsObject& object, sf::Color color)
    {
        sf::CircleShape shape(object.getRadius());
        shape.setOrigin({ object.getRadius(), object.getRadius() });
        shape.setPosition(object.getPosition());
        shape.setFillColor(color);
        shape.setOutlineColor(sf::Color::Black);
        shape.setOutlineThickness(1.f);
        return shape;
    }
} // namespace

int main()
{
    sf::RenderWindow window(sf::VideoMode({ windowWidth, windowHeight }), "Euler Integration Playground");
    window.setVerticalSyncEnabled(true);
    window.setKeyRepeatEnabled(false);

    std::cout << "Controls:\n"
        << "  Left click       - place a circle\n"
        << "  Hold left click  - grow the circle's mass before releasing it\n"
        << "  Hold w           - apply wind\n"
        << "  Space            - jump impulse for every circle\n"
        << "  Esc              - quit\n";

    // Liquid section: off the ground so ground friction can still be observed below it
    const sf::FloatRect fluidRect({ 400.f, 350.f }, { 300.f, 130.f });
    sf::RectangleShape   fluidShape(fluidRect.size);
    fluidShape.setPosition(fluidRect.position);
    fluidShape.setFillColor(sf::Color::Blue);

    std::vector<PhysicsObject> objects;

    bool         isPlacing = false;
    sf::Vector2f placePosition;
    float        placingMass = 1.f;

    sf::Clock clock;

    while (window.isOpen())
    {
        window.handleEvents(
            [&](const sf::Event::Closed&) { window.close(); },
            [&](const sf::Event::KeyPressed& keyPressed)
            {
                if (keyPressed.code == sf::Keyboard::Key::Escape)
                    window.close();
                else if (keyPressed.code == sf::Keyboard::Key::Space)
                {
                    for (auto& object : objects)
                        object.applyImpulse(
                            { 0.f, -object.getMass() * PhysicsLibrary::G_JumpImpulseStrength * PhysicsLibrary::PixelsPerMeter });
                }
            },
            [&](const sf::Event::MouseButtonPressed& mouseButtonPressed)
            {
                if (mouseButtonPressed.button == sf::Mouse::Button::Left && !isPlacing)
                {
                    isPlacing = true;
                    placingMass = 1.f;
                    placePosition = window.mapPixelToCoords(mouseButtonPressed.position);
                }
            });

        const float dt = std::min(clock.restart().asSeconds(), 0.05f);

        // Grow the circle in place while the button stays held, finalize it on release
        if (isPlacing)
        {
            if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Left))
            {
                placingMass += PhysicsLibrary::G_MassGrowthRate * dt;
            }
            else
            {
                objects.emplace_back(placingMass, placePosition);
                isPlacing = false;
            }
        }

        const bool windActive = sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W);

        for (auto& object : objects)
        {
            applyEnvironmentForces(object, fluidRect, windActive, static_cast<float>(window.getSize().y));
            object.updatePhysics(dt);
            object.collideObject(window);
        }

        window.clear(sf::Color::Black);
        window.draw(fluidShape);

        for (const auto& object : objects)
            window.draw(makeCircleShape(object, sf::Color::Green));

        if (isPlacing)
        {
            PhysicsObject preview(placingMass, placePosition);
            window.draw(makeCircleShape(preview, sf::Color::Green));
        }

        window.display();
    }
}

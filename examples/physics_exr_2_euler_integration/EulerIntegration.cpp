#include "Physics.h"

#include <SFML/Graphics.hpp>
#include <algorithm>
#include <vector>

namespace
{
    constexpr unsigned int windowWidth  = 800;
    constexpr unsigned int windowHeight = 600;

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
    sf::RenderWindow window(sf::VideoMode({ windowWidth, windowHeight }), "Euler Integration - Step 2");
    window.setVerticalSyncEnabled(true);
    window.setKeyRepeatEnabled(false);

    std::vector<PhysicsObject> objects;
    bool         isPlacing = false;
    sf::Vector2f placePosition;
    float        placingMass = 1.f;
    sf::Clock    clock;

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

        for (auto& object : objects)
        {
            object.applyForce({ 0.f, object.getMass() * PhysicsLibrary::G_Gravity * PhysicsLibrary::PixelsPerMeter });
            object.updatePhysics(dt);
            object.collideObject(window);
        }

        window.clear(sf::Color::Black);

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

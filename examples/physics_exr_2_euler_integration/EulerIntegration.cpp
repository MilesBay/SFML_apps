#include "Physics.h"
#include <SFML/Graphics.hpp>
#include <algorithm>

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
    sf::RenderWindow window(sf::VideoMode({ windowWidth, windowHeight }), "Euler Integration - Step 1");
    window.setVerticalSyncEnabled(true);

    PhysicsObject ball(1.f, { 400.f, 100.f });
    sf::Clock     clock;

    while (window.isOpen())
    {
        window.handleEvents(
            [&](const sf::Event::Closed&) { window.close(); },
            [&](const sf::Event::KeyPressed& keyPressed)
            {
                if (keyPressed.code == sf::Keyboard::Key::Escape)
                    window.close();
            });

        const float dt = std::min(clock.restart().asSeconds(), 0.05f);

        // Constant downward gravity
        ball.applyForce({ 0.f, ball.getMass() * PhysicsLibrary::G_Gravity * PhysicsLibrary::PixelsPerMeter });
        ball.updatePhysics(dt);
        ball.collideObject(window);

        window.clear(sf::Color::Black);
        window.draw(makeCircleShape(ball, sf::Color::Green));
        window.display();
    }
}

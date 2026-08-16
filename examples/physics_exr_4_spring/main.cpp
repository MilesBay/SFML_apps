#include "Physics.h"
#include <SFML/Graphics.hpp>
#include <optional>

int main() {
    constexpr unsigned int screenWidth = 800;
    constexpr unsigned int screenHeight = 600;

    sf::RenderWindow window(sf::VideoMode({ screenWidth, screenHeight }), "Particle Physics Test");
    window.setVerticalSyncEnabled(true);

    sf::Clock clock;
    const sf::Vector2f gravity(0.f, 500.f);
    Particle testParticle(sf::Vector2f(screenWidth * 0.5f, 50.f), 5.f, 0.6f);

    while (window.isOpen()) {
        float dt = clock.restart().asSeconds();
        if (dt > 0.05f) dt = 0.05f;

        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
        }

        testParticle.UpdatePhysics(dt, gravity);
        ResolveEdgeCollision(testParticle, 10.f, screenWidth, screenHeight);

        window.clear(sf::Color::Black);
        DrawParticle(window, testParticle, 10.f, sf::Color(39, 174, 96));
        window.display();
    }

    return 0;
}

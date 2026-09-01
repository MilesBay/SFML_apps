#include "BezierCurve.h"
#include "MathHelpers.h"
#include <SFML/Graphics.hpp>
#include <cassert>
#include <iostream>
#include <optional>

int main() {
    // 1. Headless boundary assertions
    CubicBezier testCurve{ {0.f, 0.f}, {0.f, 100.f}, {100.f, 100.f}, {100.f, 0.f} };
    assert(Math::Distance(testCurve.Evaluate(0.f), testCurve.p0) < 0.001f);
    assert(Math::Distance(testCurve.Evaluate(1.f), testCurve.p3) < 0.001f);
    std::cout << "Baseline math tests passed successfully.\n";

    // 2. Static Visual Smoke Test
    constexpr unsigned int screenWidth = 800;
    constexpr unsigned int screenHeight = 600;
    sf::RenderWindow window(sf::VideoMode({ screenWidth, screenHeight }), "Commit 1 - Curve Test Harness");

    const sf::Vector2f anchorLeft(60.f, screenHeight * 0.5f);
    const sf::Vector2f anchorRight(screenWidth - 60.f, screenHeight * 0.5f);
    const sf::Vector2f controlLeft(250.f, 150.f);
    const sf::Vector2f controlRight(750.f, 450.f);

    CubicBezier curve{ anchorLeft, controlLeft, controlRight, anchorRight };

    while (window.isOpen()) {
        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
        }

        window.clear(sf::Color::Black);

        DrawHandleLine(window, anchorLeft, controlLeft, sf::Color(100, 100, 100));
        DrawHandleLine(window, anchorRight, controlRight, sf::Color(100, 100, 100));
        DrawSolidCurve(window, curve, 50, sf::Color::White);

        window.display();
    }

    return 0;
}

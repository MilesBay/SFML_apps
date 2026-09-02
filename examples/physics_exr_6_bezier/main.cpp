#include "BezierCurve.h"
#include "MathHelpers.h"
#include <SFML/Graphics.hpp>
#include <SFML/Main.hpp>
#include <algorithm>
#include <cmath>
#include <optional>
#include <string>

namespace {
    sf::Vector2f ClampToWindow(sf::Vector2f p, unsigned int width, unsigned int height) {
        p.x = std::clamp(p.x, 0.f, static_cast<float>(width));
        p.y = std::clamp(p.y, 0.f, static_cast<float>(height));
        return p;
    }
}

int main() {
    constexpr unsigned int screenWidth = 800;
    constexpr unsigned int screenHeight = 600;
    constexpr float travelTimeSeconds = 2.f; // time for the demo circle to cross the curve one-way

    sf::RenderWindow window(sf::VideoMode({ screenWidth, screenHeight }), "Cubic Bezier Curve Demo");
    window.setVerticalSyncEnabled(true);

    sf::Font font;
    bool fontLoaded = font.openFromFile("../resources/tuffy.ttf");
    std::optional<sf::Text> uiText;
    if (fontLoaded) {
        uiText.emplace(font);
        uiText->setCharacterSize(14);
        uiText->setFillColor(sf::Color::White);
    }

    // Anchor points sit near the left/right edges and never move.
    const sf::Vector2f anchorLeft(60.f, screenHeight * 0.5f);
    const sf::Vector2f anchorRight(screenWidth - 60.f, screenHeight * 0.5f);

    // Control points are user-draggable: left mouse button moves the left one, right mouse button the right one.
    sf::Vector2f controlLeft(250.f, 150.f);
    sf::Vector2f controlRight(750.f, 450.f);

    sf::Clock motionClock;

    while (window.isOpen()) {
        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
        }

        if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Left)) {
            controlLeft = ClampToWindow(sf::Vector2f(sf::Mouse::getPosition(window)), screenWidth, screenHeight);
        }
        if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Right)) {
            controlRight = ClampToWindow(sf::Vector2f(sf::Mouse::getPosition(window)), screenWidth, screenHeight);
        }

        CubicBezier curve{ anchorLeft, controlLeft, controlRight, anchorRight };

        // Drive a circle back and forth along the curve, easing with sine so it
        // accelerates out of and decelerates into each end rather than moving linearly.
        float elapsed = motionClock.getElapsedTime().asSeconds();
        float loopedTime = std::fmod(elapsed, travelTimeSeconds * 2.f);
        float phase = (loopedTime / travelTimeSeconds) * Math::PI;
        float t = 0.5f + 0.5f * std::sin(phase - Math::PI * 0.5f);
        sf::Vector2f movingPos = curve.Evaluate(t);

        window.clear(sf::Color::Black);

        DrawHandleLine(window, anchorLeft, controlLeft, sf::Color::White);
        DrawHandleLine(window, anchorRight, controlRight, sf::Color::White);

        DrawDashedCurve(window, curve, 80, 4.f, sf::Color::White);

        sf::CircleShape anchorShape(8.f);
        anchorShape.setOrigin({ 8.f, 8.f });
        anchorShape.setFillColor(sf::Color::White);
        anchorShape.setPosition(anchorLeft);
        window.draw(anchorShape);
        anchorShape.setPosition(anchorRight);
        window.draw(anchorShape);

        sf::CircleShape controlShape(10.f);
        controlShape.setOrigin({ 10.f, 10.f });
        controlShape.setOutlineColor(sf::Color::White);
        controlShape.setOutlineThickness(1.5f);

        controlShape.setFillColor(sf::Color::Blue); // blue = left mouse button
        controlShape.setPosition(controlLeft);
        window.draw(controlShape);

        controlShape.setFillColor(sf::Color::Red); // red = right mouse button
        controlShape.setPosition(controlRight);
        window.draw(controlShape);

        sf::CircleShape movingShape(9.f);
        movingShape.setOrigin({ 9.f, 9.f });
        movingShape.setFillColor(sf::Color::Green);
        movingShape.setPosition(movingPos);
        window.draw(movingShape);

        if (fontLoaded) {
            uiText->setString(
                "Left Mouse: drag left (blue) control point\n"
                "Right Mouse: drag right (red) control point");
            uiText->setPosition({ 15.f, 15.f });
            window.draw(*uiText);
        }

        window.display();
    }

    return 0;
}

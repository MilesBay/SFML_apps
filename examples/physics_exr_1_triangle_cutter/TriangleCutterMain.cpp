#include <SFML/Graphics.hpp>
#include <SFML/Main.hpp>
#include <array>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include "Plane2d.h"

namespace
{
    std::filesystem::path resourcesDir()
    {
        return "../resources";
    }

    constexpr unsigned int windowWidth = 800;
    constexpr unsigned int windowHeight = 600;

    sf::ConvexShape makeTriangleShape(const sf::Vector2f& a, const sf::Vector2f& b, const sf::Vector2f& c, const sf::Color& color)
    {
        sf::ConvexShape triangle(3);
        triangle.setPoint(0, a);
        triangle.setPoint(1, b);
        triangle.setPoint(2, c);
        triangle.setFillColor(color);
        triangle.setOutlineColor(sf::Color::White);
        triangle.setOutlineThickness(1.f);
        return triangle;
    }

    sf::CircleShape makeMarker(const sf::Vector2f& position, const sf::Color& color)
    {
        constexpr float radius = 4.f;
        sf::CircleShape marker(radius);
        marker.setOrigin({ radius, radius });
        marker.setPosition(position);
        marker.setFillColor(color);
        return marker;
    }

    // Return a string label for the side of the plane based on the distance
    std::string sideLabel(float distance)
    {
        if (distance > 0.f)
            return "in front of";
        if (distance < 0.f)
            return "behind";
        return "on";
    }
} // namespace

int main()
{
    sf::RenderWindow window(sf::VideoMode({ windowWidth, windowHeight }), "Triangle Cutting");
    window.setVerticalSyncEnabled(true);

    const std::string instructions =
        "Left click x3: place a triangle (further clicks reset it)\n"
        "Right click x2: place a cut plane, point then direction (further clicks reset it)";

    const sf::Font font(resourcesDir() / "tuffy.ttf");
    sf::Text osdText(font, instructions, 16u);
    osdText.setFillColor(sf::Color::White);
    osdText.setPosition({ 10.f, 10.f });

    std::vector<sf::Vector2f> trianglePoints;
    std::vector<sf::Vector2f> planePoints;

    while (window.isOpen())
    {
        window.handleEvents(
            [&](const sf::Event::Closed&) { window.close(); },
            [&](const sf::Event::KeyPressed& keyPress)
            {
                if (keyPress.code == sf::Keyboard::Key::Escape)
                    window.close();
            },
            [&](const sf::Event::MouseButtonPressed& mouseButtonPressed)
            {
                const auto worldPosition = window.mapPixelToCoords(mouseButtonPressed.position);

                if (mouseButtonPressed.button == sf::Mouse::Button::Left)
                {
                    if (trianglePoints.size() == 3)
                        trianglePoints.clear();
                    trianglePoints.push_back(worldPosition);
                }
                else if (mouseButtonPressed.button == sf::Mouse::Button::Right)
                {
                    if (planePoints.size() == 2)
                        planePoints.clear();
                    planePoints.push_back(worldPosition);
                }
            });

        window.clear(sf::Color::Black);

        const bool haveTriangle = trianglePoints.size() == 3;
        const bool havePlane = planePoints.size() == 2;

        if (haveTriangle && havePlane)
        {
            const std::array<sf::Vector2f, 3> vertices{ trianglePoints[0], trianglePoints[1], trianglePoints[2] };
            const sf::Vector2f direction = (planePoints[1] - planePoints[0]).normalized();
            const Plane2d plane{ planePoints[0], sf::Vector2f{-direction.y, direction.x} };

            window.draw(makeTriangleShape(vertices[0], vertices[1], vertices[2], sf::Color::White));

            std::string status = instructions + "\n";
            for (std::size_t i = 0; i < vertices.size(); ++i)
            {
                std::string s = "\nPoint " + std::to_string(i) + " is " + sideLabel(plane.signedDistance(vertices[i])) + " the plane";
                std::cout << s << std::endl;
                status += s;
            }
            osdText.setString(status);
        }
        else if (haveTriangle)
        {
            window.draw(makeTriangleShape(trianglePoints[0], trianglePoints[1], trianglePoints[2], sf::Color::White));
            osdText.setString(instructions);
        }

        if (trianglePoints.size() == 3)
        {
            window.draw(makeTriangleShape(trianglePoints[0], trianglePoints[1], trianglePoints[2], sf::Color::White));
        }
        else
        {
            for (const auto& point : trianglePoints)
                window.draw(makeMarker(point, sf::Color::Yellow));
        }

        if (planePoints.size() == 2)
        {
            const sf::Vector2f direction = (planePoints[1] - planePoints[0]).normalized();
            const sf::Vector2f farBack = planePoints[0] - direction * 2000.f;
            const sf::Vector2f farFront = planePoints[0] + direction * 2000.f;

            const std::array<sf::Vertex, 2> line{ sf::Vertex{farBack}, sf::Vertex{farFront} };
            window.draw(line.data(), line.size(), sf::PrimitiveType::Lines);
        }
        else
        {
            for (const auto& point : planePoints)
                window.draw(makeMarker(point, sf::Color::Cyan));
        }

        osdText.setString(instructions);
        window.draw(osdText);
        window.display();
    }
}
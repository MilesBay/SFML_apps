#include "Physics.h"
#include <SFML/Graphics.hpp>
#include <SFML/Main.hpp>
#include <vector>
#include <string>
#include <optional>
#include <sstream>
#include <iomanip>

int main() {
    constexpr unsigned int screenWidth = 800;
    constexpr unsigned int screenHeight = 600;

    sf::RenderWindow window(sf::VideoMode({ screenWidth, screenHeight }), "Rope Simulation - Mass-Spring Chain");
    window.setVerticalSyncEnabled(true);
    window.setKeyRepeatEnabled(false);

    sf::Clock clock;

    sf::Font font;
    bool fontLoaded = font.openFromFile("../resources/tuffy.ttf");
    std::optional<sf::Text> uiText;
    if (fontLoaded) {
        uiText.emplace(font);
        uiText->setCharacterSize(14);
        uiText->setFillColor(sf::Color::White);
    }

    // --- Rope configuration ---
    constexpr int particleCount = 10;
    constexpr float segmentLength = 30.f;
    constexpr float minMass = 1.f;
    constexpr float maxMass = 10.f;
    constexpr float massRatePerSecond = 4.f;
    constexpr float springStiffness = 4000.f;
    constexpr float springDamping = 40.f;
    constexpr int substeps = 8;
    float currentMass = 5.f;

    std::vector<Particle> particles;
    particles.reserve(particleCount);
    sf::Vector2f startPos(screenWidth * 0.5f - (particleCount - 1) * segmentLength * 0.5f, 100.f);
    for (int i = 0; i < particleCount; ++i) {
        particles.push_back(Particle(startPos + sf::Vector2f(segmentLength * static_cast<float>(i), 0.f), currentMass, 0.5f));
    }

    std::vector<Spring> springs;
    springs.reserve(particleCount - 1);
    for (int i = 0; i < particleCount - 1; ++i) {
        Spring s;
        s.indexA = static_cast<size_t>(i);
        s.indexB = static_cast<size_t>(i + 1);
        s.restLength = segmentLength;
        s.stiffness = springStiffness;
        s.damping = springDamping;
        springs.push_back(s);
    }

    const sf::Vector2f gravity(0.f, 500.f);

    bool draggingFirst = false;
    sf::Vector2f dragTargetPos = particles[0].position;

    while (window.isOpen()) {
        float dt = clock.restart().asSeconds();
        if (dt > 0.05f) dt = 0.05f;

        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }

            // +12px pads the grab radius past the drawn particle so small/light masses stay easy to click
            if (const auto* mouseButtonPressed = event->getIf<sf::Event::MouseButtonPressed>()) {
                if (mouseButtonPressed->button == sf::Mouse::Button::Left) {
                    sf::Vector2f mousePos(static_cast<float>(mouseButtonPressed->position.x), static_cast<float>(mouseButtonPressed->position.y));
                    float grabRadius = MassToRadius(particles[0].mass, minMass, maxMass) + 12.f;
                    if (Math::Distance(mousePos, particles[0].position) <= grabRadius) {
                        draggingFirst = true;
                        dragTargetPos = mousePos;
                    }
                }
            }

            if (const auto* mouseButtonReleased = event->getIf<sf::Event::MouseButtonReleased>()) {
                if (mouseButtonReleased->button == sf::Mouse::Button::Left) {
                    draggingFirst = false;
                }
            }

            if (const auto* mouseMoved = event->getIf<sf::Event::MouseMoved>()) {
                if (draggingFirst) {
                    dragTargetPos = sf::Vector2f(static_cast<float>(mouseMoved->position.x), static_cast<float>(mouseMoved->position.y));
                }
            }
        }

        // Hold M / N to grow or shrink the mass of every particle, clamped to [minMass, maxMass]
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::M)) {
            currentMass += massRatePerSecond * dt;
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::N)) {
            currentMass -= massRatePerSecond * dt;
        }
        currentMass = std::max(minMass, std::min(maxMass, currentMass));
        for (auto& p : particles) {
            p.mass = currentMass;
        }

        sf::Vector2f windForce(0.f, 0.f);
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W)) {
            windForce = sf::Vector2f(250.f, 0.f);
        }

        float particleRadius = MassToRadius(currentMass, minMass, maxMass);
        float ropeThickness = MassToThickness(currentMass, minMass, maxMass);

        // Dragging directly drives particle 0's kinematics for this frame; velocity is
        // derived from the mouse delta so the rope flings naturally on release.
        if (draggingFirst) {
            if (dt > 0.0001f) {
                particles[0].velocity = (dragTargetPos - particles[0].position) / dt;
            }
            particles[0].position = dragTargetPos;
        }

        // Fixed substeps keep the springs stable regardless of frame-rate dt
        float subDt = dt / static_cast<float>(substeps);
        for (int step = 0; step < substeps; ++step) {
            // 1. Accumulate external force for this sub-step (wind)
            for (auto& p : particles) {
                p.ApplyForce(windForce * p.mass);
            }

            // 2. Accumulate internal force for this sub-step (spring pulling each segment back to restLength)
            for (auto& s : springs) {
                ApplySpringForce(particles, s);
            }

            // 3. Integrate accumulated force + gravity into new velocity/position.
            //    Particle 0 skips this while dragged since the mouse already drives its position/velocity directly.
            for (size_t i = 0; i < particles.size(); ++i) {
                if (draggingFirst && i == 0) {
                    particles[0].force = sf::Vector2f(0.f, 0.f);
                    continue;
                }
                particles[i].UpdatePhysics(subDt, gravity);
            }

            // 4. Keep particles inside the window (same drag exemption as above)
            for (size_t i = 0; i < particles.size(); ++i) {
                if (draggingFirst && i == 0) continue;
                ResolveEdgeCollision(particles[i], particleRadius, static_cast<float>(screenWidth), static_cast<float>(screenHeight));
            }
        }

        window.clear(sf::Color::Black);

        if (fontLoaded) {
            std::ostringstream massStream;
            massStream << std::fixed << std::setprecision(1) << currentMass;

            std::string hudString = "Rope Simulation | Mass: " + massStream.str() + " (M: +, N: -)\n";
            hudString += "Left Click + Drag first particle to move it. Hold W for wind.";
            uiText->setString(hudString);
            uiText->setPosition({ 15.f, 15.f });
            window.draw(*uiText);
        }

        for (size_t i = 0; i + 1 < particles.size(); ++i) {
            DrawRopeSegment(window, particles[i].position, particles[i + 1].position, ropeThickness, sf::Color(46, 204, 113));
        }
        for (const auto& p : particles) {
            DrawParticle(window, p, particleRadius, sf::Color(39, 174, 96));
        }

        window.display();
    }

    return 0;
}

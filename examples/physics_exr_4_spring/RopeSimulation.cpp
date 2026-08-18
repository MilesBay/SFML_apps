#include "Physics.h"
#include <SFML/Graphics.hpp>
#include <vector>
#include <optional>

int main() {
    constexpr unsigned int screenWidth = 800;
    constexpr unsigned int screenHeight = 600;

    sf::RenderWindow window(sf::VideoMode({ screenWidth, screenHeight }), "Rope Simulation - Mass-Spring Chain");
    window.setVerticalSyncEnabled(true);

    sf::Clock clock;

    constexpr int particleCount = 10;
    constexpr float segmentLength = 30.f;
    constexpr float minMass = 1.f;
    constexpr float maxMass = 10.f;
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

    while (window.isOpen()) {
        float dt = clock.restart().asSeconds();
        if (dt > 0.05f) dt = 0.05f;

        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
        }

        float particleRadius = MassToRadius(currentMass, minMass, maxMass);
        float ropeThickness = MassToThickness(currentMass, minMass, maxMass);

        float subDt = dt / static_cast<float>(substeps);
        for (int step = 0; step < substeps; ++step) {
            for (auto& s : springs) {
                ApplySpringForce(particles, s);
            }
            // Anchor particle 0 in place for initial stability check
            for (size_t i = 1; i < particles.size(); ++i) {
                particles[i].UpdatePhysics(subDt, gravity);
                ResolveEdgeCollision(particles[i], particleRadius, static_cast<float>(screenWidth), static_cast<float>(screenHeight));
            }
        }

        window.clear(sf::Color::Black);

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

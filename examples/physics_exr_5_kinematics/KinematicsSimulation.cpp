#include "Kinematics.h"
#include <SFML/Graphics.hpp>
#include <SFML/Main.hpp>
#include <vector>
#include <optional>
#include <random>

struct GrassBlade {
    KinematicChain chain;
    std::vector<float> segmentWeight;
    std::vector<float> localAngles;
    float swayFreq = 0.f;
    float swayPhase = 0.f;
    float swayAmplitude = 0.f;
};

struct Worm {
    KinematicChain chain;
};

int main() {
    constexpr unsigned int screenWidth = 1280;
    constexpr unsigned int screenHeight = 960;

    sf::RenderWindow window(sf::VideoMode({ screenWidth, screenHeight }), "Test Commit 2 - Head Follow IK Worm");
    window.setVerticalSyncEnabled(true);

    sf::Clock clock;
    float elapsedTime = 0.f;

    // Grass setup
    constexpr int grassBladeCount = 80;
    constexpr int grassSegmentCount = 6;
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> xJitterDist(-8.f, 8.f);
    std::uniform_real_distribution<float> segLenDist(5.f, 20.f);
    std::uniform_real_distribution<float> freqDist(0.6f, 1.4f);
    std::uniform_real_distribution<float> phaseDist(0.f, Math::PI * 2.f);
    std::uniform_real_distribution<float> ampDist(0.10f, 0.22f);

    std::vector<GrassBlade> grassBlades;
    for (int i = 0; i < grassBladeCount; ++i) {
        float t = static_cast<float>(i) / static_cast<float>(grassBladeCount - 1);
        float x = Math::Lerp(25.f, static_cast<float>(screenWidth) - 25.f, t) + xJitterDist(rng);
        GrassBlade blade{ KinematicChain(std::vector<float>(grassSegmentCount, segLenDist(rng)), {x, static_cast<float>(screenHeight)}) };
        blade.swayFreq = freqDist(rng);
        blade.swayPhase = phaseDist(rng);
        blade.swayAmplitude = ampDist(rng);
        blade.segmentWeight.resize(grassSegmentCount);
        blade.localAngles.resize(grassSegmentCount, 0.f);
        for (int s = 0; s < grassSegmentCount; ++s) {
            blade.segmentWeight[s] = 0.3f + 0.7f * (static_cast<float>(s) / static_cast<float>(grassSegmentCount - 1));
        }
        grassBlades.push_back(std::move(blade));
    }

    // --- Worm setup: Follow IK test ---
    constexpr int wormSegmentCount = 6;
    constexpr float wormSegmentLength = 42.f;
    std::vector<float> wormLengths(wormSegmentCount, wormSegmentLength);
    Worm worm{ KinematicChain(wormLengths, sf::Vector2f(static_cast<float>(screenWidth) * 0.25f, static_cast<float>(screenHeight) * 0.25f), sf::Vector2f(0.f, 1.f)) };

    while (window.isOpen()) {
        float dt = clock.restart().asSeconds();
        if (dt > 0.05f) dt = 0.05f;
        elapsedTime += dt;

        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
        }

        sf::Vector2i mousePixel = sf::Mouse::getPosition(window);
        sf::Vector2f mousePos(static_cast<float>(mousePixel.x), static_cast<float>(mousePixel.y));

        for (auto& blade : grassBlades) {
            float wave = std::sin(elapsedTime * blade.swayFreq + blade.swayPhase);
            for (size_t s = 0; s < blade.localAngles.size(); ++s) {
                blade.localAngles[s] = blade.swayAmplitude * blade.segmentWeight[s] * wave;
            }
            SolveForwardKinematics(blade.chain, blade.chain.joints[0], blade.localAngles);
        }

        // Run Head Follow IK test
        SolveHeadFollowIK(worm.chain, mousePos);

        window.clear(sf::Color::Black);
        for (const auto& blade : grassBlades) {
            DrawChain(window, blade.chain, 6.f, 1.5f, sf::Color(40, 110, 45), sf::Color(120, 210, 90));
        }
        DrawChain(window, worm.chain, 14.f, 4.f, sf::Color::Green, sf::Color::White);
        DrawJoint(window, worm.chain.joints.front(), 10.f, sf::Color::White);
        window.display();
    }

    return 0;
}

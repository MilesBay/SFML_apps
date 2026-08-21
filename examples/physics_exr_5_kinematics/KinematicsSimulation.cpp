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

int main() {
    constexpr unsigned int screenWidth = 1280;
    constexpr unsigned int screenHeight = 960;

    sf::RenderWindow window(sf::VideoMode({ screenWidth, screenHeight }), "Test Commit 1 - Forward Kinematics Grass");
    window.setVerticalSyncEnabled(true);

    sf::Clock clock;
    float elapsedTime = 0.f;

    // --- Grass field: Forward Kinematics Test ---
    constexpr int grassBladeCount = 100;
    constexpr int grassSegmentCount = 6;

    std::mt19937 rng(1337); // deterministic seed for test stability
    std::uniform_real_distribution<float> xJitterDist(-8.f, 8.f);
    std::uniform_real_distribution<float> segLenDist(5.f, 20.f);
    std::uniform_real_distribution<float> freqDist(0.6f, 1.4f);
    std::uniform_real_distribution<float> phaseDist(0.f, Math::PI * 2.f);
    std::uniform_real_distribution<float> ampDist(0.10f, 0.22f);

    std::vector<GrassBlade> grassBlades;
    grassBlades.reserve(grassBladeCount);
    for (int i = 0; i < grassBladeCount; ++i) {
        float t = static_cast<float>(i) / static_cast<float>(grassBladeCount - 1);
        float x = Math::Lerp(25.f, static_cast<float>(screenWidth) - 25.f, t) + xJitterDist(rng);
        float segLen = segLenDist(rng);

        std::vector<float> segmentLengths(grassSegmentCount, segLen);
        sf::Vector2f origin(x, static_cast<float>(screenHeight));

        GrassBlade blade{ KinematicChain(segmentLengths, origin, sf::Vector2f(0.f, -1.f)) };
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

    while (window.isOpen()) {
        float dt = clock.restart().asSeconds();
        if (dt > 0.05f) dt = 0.05f;
        elapsedTime += dt;

        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
        }

        // Test FK sway update
        for (auto& blade : grassBlades) {
            float wave = std::sin(elapsedTime * blade.swayFreq + blade.swayPhase);
            for (size_t s = 0; s < blade.localAngles.size(); ++s) {
                blade.localAngles[s] = blade.swayAmplitude * blade.segmentWeight[s] * wave;
            }
            SolveForwardKinematics(blade.chain, blade.chain.joints[0], blade.localAngles);
        }

        window.clear(sf::Color::Black);
        for (const auto& blade : grassBlades) {
            DrawChain(window, blade.chain, 6.f, 1.5f, sf::Color(40, 110, 45), sf::Color(120, 210, 90));
        }
        window.display();
    }

    return 0;
}

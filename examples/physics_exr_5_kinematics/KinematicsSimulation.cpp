#include "Kinematics.h"
#include <SFML/Graphics.hpp>
#include <SFML/Main.hpp>
#include <vector>
#include <optional>
#include <random>

// One grass blade: a Forward Kinematics chain rooted at the ground.
struct GrassBlade {
    KinematicChain chain;
    std::vector<float> segmentWeight;  // per-segment share of the sway (root barely bends, tip bends most)
    std::vector<float> localAngles;    // scratch buffer rebuilt every frame before solving
    float swayFreq = 0.f;
    float swayPhase = 0.f;
    float swayAmplitude = 0.f;
};

// The arm: an Inverse Kinematics (FABRIK) chain, root pinned to a fixed base.
struct Arm {
    KinematicChain chain;
    sf::Vector2f base;
};

// The worm: an Inverse Kinematics chain with a free tail, head pinned to a moving target.
struct Worm {
    KinematicChain chain;
};

int main() {
    constexpr unsigned int screenWidth = 1280;
    constexpr unsigned int screenHeight = 960;

    sf::RenderWindow window(sf::VideoMode({ screenWidth, screenHeight }), "Kinematics - Grass, Arm & Worm");
    window.setVerticalSyncEnabled(true);

    sf::Clock clock;
    float elapsedTime = 0.f;

    sf::Font font;
    bool fontLoaded = font.openFromFile("../resources/tuffy.ttf");
    std::optional<sf::Text> uiText;
    if (fontLoaded) {
        uiText.emplace(font);
        uiText->setCharacterSize(14);
        uiText->setFillColor(sf::Color::White);
    }

    // --- Grass field: Forward Kinematics ---
    constexpr int grassBladeCount = 100;  // >=50
    constexpr int grassSegmentCount = 6;  // >=5

    std::mt19937 rng(std::random_device{}());
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

    // --- Arm: Inverse Kinematics (FABRIK), root pinned to the window centre ---
    constexpr int armSegmentCount = 6;
    constexpr float armSegmentLength = 42.f;
    std::vector<float> armLengths(armSegmentCount, armSegmentLength);
    Arm arm{ KinematicChain(armLengths, sf::Vector2f(static_cast<float>(screenWidth) * 0.5f, static_cast<float>(screenHeight) * 0.5f), sf::Vector2f(1.f, 0.f)) };
    arm.base = arm.chain.joints[0];

    // --- Worm: Inverse Kinematics, free tail, head pinned to the mouse ---
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

        // Grass sway: rebuild each blade's local bend angles from its own
        // frequency/phase/amplitude, then run Forward Kinematics.
        for (auto& blade : grassBlades) {
            float wave = std::sin(elapsedTime * blade.swayFreq + blade.swayPhase);
            for (size_t s = 0; s < blade.localAngles.size(); ++s) {
                blade.localAngles[s] = blade.swayAmplitude * blade.segmentWeight[s] * wave;
            }
            SolveForwardKinematics(blade.chain, blade.chain.joints[0], blade.localAngles);
        }

        // Arm reaches for the mouse while its root stays anchored at the window centre.
        SolveFABRIK(arm.chain, arm.base, mousePos);

        // Worm's head is pinned directly to the mouse; the free tail trails behind it.
        SolveHeadFollowIK(worm.chain, mousePos);

        window.clear(sf::Color::Black);

        for (const auto& blade : grassBlades) {
            DrawChain(window, blade.chain, 6.f, 1.5f, sf::Color(40, 110, 45), sf::Color(120, 210, 90));
        }

        DrawChain(window, worm.chain, 14.f, 4.f, sf::Color::Green, sf::Color::White);
        DrawJoint(window, worm.chain.joints.front(), 10.f, sf::Color::White);

        DrawChain(window, arm.chain, 16.f, 6.f, sf::Color::Black, sf::Color::White);
        for (const auto& joint : arm.chain.joints) {
            DrawJoint(window, joint, 6.f, sf::Color::White);
        }
        DrawJoint(window, arm.chain.joints.front(), 11.f, sf::Color::White);
        DrawJoint(window, arm.chain.joints.back(), 7.f, sf::Color::Yellow);

        if (fontLoaded) {
            uiText->setString(
                "Kinematics Demo\n"
                "Grass (Forward Kinematics) sways on its own.\n"
                "Arm (Inverse Kinematics, anchored centre) and Worm (Inverse Kinematics, free) both reach for the mouse.");
            uiText->setPosition({ 15.f, 15.f });
            window.draw(*uiText);
        }

        window.display();
    }

    return 0;
}

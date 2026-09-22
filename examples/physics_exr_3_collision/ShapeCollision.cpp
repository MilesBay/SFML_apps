#include "Physics.h"
#include <SFML/Graphics.hpp>
#include <SFML/Main.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <optional>

// Interactive demo/test harness for the physics & collision code in
// Physics.h/Physics.cpp. Presents three independently-switchable "scenes",
// each exercising a different collision primitive:
//   Scene 1 - Capsule vs. capsule collision (ResolveCapsuleCollision)
//   Scene 2 - Convex polygon vs. convex polygon collision via SAT
//             (ResolveConvexCollision)
//   Scene 3 - Point-in-triangle overlap test (IsPointInTriangle) used to
//             repel spawned circles out of a static triangular zone
// All three scenes share the same basic per-frame loop: apply forces
// (gravity + optional wind) -> integrate physics -> resolve collisions ->
// clamp to screen bounds -> draw.
int main() {
    constexpr unsigned int screenWidth = 800;
    constexpr unsigned int screenHeight = 600;

    sf::RenderWindow window(sf::VideoMode({ screenWidth, screenHeight }), "MDS Shape Collision Engine");
    window.setVerticalSyncEnabled(true);   // Cap frame rate to the display's refresh rate (also keeps dt reasonably small/stable).
    window.setKeyRepeatEnabled(false);     // Avoid duplicate KeyPressed events while a key is held down.

    sf::Clock clock;
    int currentScene = 1;   // Which of the 3 demo scenes is currently active; switched via the 1/2/3 keys below.

    // Best-effort font load for the on-screen HUD text; the HUD is simply
    // skipped if the font can't be found so the demo still runs without it.
    sf::Font font;
    bool fontLoaded = font.openFromFile("../resources/tuffy.ttf");
    std::optional<sf::Text> uiText;
    if (fontLoaded) {
        uiText.emplace(font);
        uiText->setCharacterSize(14);
        uiText->setFillColor(sf::Color::White);
    }

    // SCENE 1 setup: five capsules of varying length/radius/mass/initial
    // angle, scattered around the window, each with a distinct color.
    std::vector<Capsule> capsules;
    capsules.push_back(Capsule(sf::Vector2f(200.f, 200.f), 120.f, 20.f, 0.4f, 6.0f, 0.6f, sf::Color(46, 204, 113)));
    capsules.push_back(Capsule(sf::Vector2f(450.f, 150.f), 150.f, 25.f, 1.2f, 9.0f, 0.6f, sf::Color(52, 152, 219)));
    capsules.push_back(Capsule(sf::Vector2f(600.f, 350.f), 100.f, 18.f, 2.5f, 4.5f, 0.6f, sf::Color(155, 89, 182)));
    capsules.push_back(Capsule(sf::Vector2f(150.f, 450.f), 140.f, 22.f, -0.8f, 7.5f, 0.6f, sf::Color(230, 126, 34)));
    capsules.push_back(Capsule(sf::Vector2f(350.f, 400.f), 110.f, 24.f, 1.8f, 6.5f, 0.6f, sf::Color(241, 196, 15)));

    // SCENE 2 setup: build reusable local-space vertex lists for a
    // triangle, a regular pentagon, and a regular hexagon (each centered on
    // its own local origin), then instantiate 5 ConvexBody shapes that
    // reuse these vertex sets at different world positions/rotations.
    std::vector<ConvexBody> convexShapes;
    std::vector<sf::Vector2f> localTriangle = {
        {-40.f, 40.f},
        {40.f, 40.f},
        {0.f, -50.f}
    };
    // Regular pentagon: 5 vertices evenly spaced around a circle of radius
    // 45, offset by -PI/2 so one vertex points straight up.
    std::vector<sf::Vector2f> localPentagon;
    for (int i = 0; i < 5; ++i) {
        float a = i * 2.f * Math::PI / 5.f - Math::PI / 2.f;
        localPentagon.push_back({ 45.f * std::cos(a), 45.f * std::sin(a) });
    }
    // Regular hexagon: 6 vertices evenly spaced around a circle of radius 50.
    std::vector<sf::Vector2f> localHexagon;
    for (int i = 0; i < 6; ++i) {
        float a = i * 2.f * Math::PI / 6.f;
        localHexagon.push_back({ 50.f * std::cos(a), 50.f * std::sin(a) });
    }

    convexShapes.push_back(ConvexBody(sf::Vector2f(200.f, 200.f), localTriangle, 0.2f, 4.0f, 0.5f, sf::Color(231, 76, 60)));
    convexShapes.push_back(ConvexBody(sf::Vector2f(400.f, 150.f), localPentagon, 1.0f, 6.0f, 0.5f, sf::Color(155, 89, 182)));
    convexShapes.push_back(ConvexBody(sf::Vector2f(600.f, 250.f), localHexagon, 2.1f, 8.0f, 0.5f, sf::Color(26, 188, 156)));
    convexShapes.push_back(ConvexBody(sf::Vector2f(150.f, 450.f), localPentagon, -0.5f, 6.0f, 0.5f, sf::Color(52, 152, 219)));
    convexShapes.push_back(ConvexBody(sf::Vector2f(450.f, 450.f), localHexagon, 3.0f, 8.0f, 0.5f, sf::Color(241, 196, 15)));

    // SCENE 3 setup: a static triangular "danger zone" (fixed, non-physics
    // vertices) plus its centroid (used later as a fallback push-away
    // origin), and an initially-empty list of circles that the user spawns
    // by clicking.
    const sf::Vector2f triA(400.f, 150.f);
    const sf::Vector2f triB(250.f, 400.f);
    const sf::Vector2f triC(550.f, 400.f);
    const sf::Vector2f triangleCentroid = (triA + triB + triC) / 3.f;

    std::vector<CircleBody> circles;
    const sf::Vector2f gravity(0.f, 400.f);   // Downward gravitational acceleration applied to every body, every scene.

    // --- Main loop ---
    while (window.isOpen()) {
        // Delta time since last frame, clamped to 0.05s (20 FPS worth) to
        // prevent huge integration steps (and the instability/tunneling
        // they'd cause) if the app stalls or a breakpoint is hit.
        float dt = clock.restart().asSeconds();
        if (dt > 0.05f) dt = 0.05f;

        // --- Event handling ---
        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }

            // Number keys 1/2/3 switch between the three demo scenes.
            if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>()) {
                if (keyPressed->code == sf::Keyboard::Key::Num1) {
                    currentScene = 1;
                }
                else if (keyPressed->code == sf::Keyboard::Key::Num2) {
                    currentScene = 2;
                }
                else if (keyPressed->code == sf::Keyboard::Key::Num3) {
                    currentScene = 3;
                }
            }

            // In Scene 3 only: left-click spawns a new CircleBody at the
            // mouse position (mass 3.0, restitution 0.7).
            if (currentScene == 3) {
                if (const auto* mouseButtonPressed = event->getIf<sf::Event::MouseButtonPressed>()) {
                    if (mouseButtonPressed->button == sf::Mouse::Button::Left) {
                        sf::Vector2f mousePos(static_cast<float>(mouseButtonPressed->position.x), static_cast<float>(mouseButtonPressed->position.y));
                        circles.push_back(CircleBody(mousePos, 20.f, 3.0f, 0.7f, sf::Color(230, 126, 34)));
                    }
                }
            }
        }

        // Holding 'W' applies a constant rightward wind force to every body
        // in the active scene (scaled by each body's mass when applied, so
        // it produces uniform acceleration regardless of mass, like gravity).
        sf::Vector2f windForce(0.f, 0.f);
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W)) {
            windForce = sf::Vector2f(250.f, 0.f);
        }

        if (currentScene == 1) {
            // 1. Integrate: apply wind (scaled by mass) + gravity, advance
            //    linear motion, and spin each capsule at its fixed angular speed.
            for (auto& cap : capsules) {
                cap.ApplyForce(windForce * cap.mass);
                cap.UpdatePhysics(dt, gravity);
                cap.UpdateRotation(dt);
            }

            // 2. Resolve collisions: test every unique pair of capsules
            //    (i < j avoids testing a pair twice or a capsule against itself).
            for (size_t i = 0; i < capsules.size(); ++i) {
                for (size_t j = i + 1; j < capsules.size(); ++j) {
                    ResolveCapsuleCollision(capsules[i], capsules[j]);
                }
            }

            // 3. Clamp to screen bounds: compute each capsule's axis-aligned
            //    bounding box (from its two endpoints +/- radius) and, if it
            //    pokes past an edge, push it back in and reflect the
            //    corresponding velocity component (scaled by restitution
            //    for bounciness). Each axis is handled independently and
            //    treated as mutually exclusive (checked via else-if), which
            //    is fine since the capsule can only be clamped from one side
            //    of the screen per axis per frame.
            for (auto& cap : capsules) {
                sf::Vector2f A = cap.GetWorldA();
                sf::Vector2f B = cap.GetWorldB();
                float r = cap.radius;

                float minX = std::min(A.x, B.x) - r;
                float maxX = std::max(A.x, B.x) + r;
                if (minX < 0.f) {
                    cap.position.x += -minX;
                    cap.velocity.x = std::abs(cap.velocity.x) * cap.restitution;
                }
                else if (maxX > screenWidth) {
                    cap.position.x -= (maxX - screenWidth);
                    cap.velocity.x = -std::abs(cap.velocity.x) * cap.restitution;
                }

                float minY = std::min(A.y, B.y) - r;
                float maxY = std::max(A.y, B.y) + r;
                if (minY < 0.f) {
                    cap.position.y += -minY;
                    cap.velocity.y = std::abs(cap.velocity.y) * cap.restitution;
                }
                else if (maxY > screenHeight) {
                    cap.position.y -= (maxY - screenHeight);
                    cap.velocity.y = -std::abs(cap.velocity.y) * cap.restitution;
                }
            }
        }
        else if (currentScene == 2) {
            // 1. Integrate: same wind/gravity/rotation treatment as Scene 1,
            //    applied to each convex polygon body.
            for (auto& body : convexShapes) {
                body.ApplyForce(windForce * body.mass);
                body.UpdatePhysics(dt, gravity);
                body.UpdateRotation(dt);
            }

            // 2. Resolve collisions: SAT-test every unique pair of polygons.
            for (size_t i = 0; i < convexShapes.size(); ++i) {
                for (size_t j = i + 1; j < convexShapes.size(); ++j) {
                    ResolveConvexCollision(convexShapes[i], convexShapes[j]);
                }
            }

            // 3. Clamp to screen bounds: compute each polygon's world-space
            //    axis-aligned bounding box from its actual vertices (rather
            //    than a fixed radius, since polygons aren't circularly
            //    symmetric) and push/reflect exactly as in Scene 1.
            for (auto& body : convexShapes) {
                auto verts = body.GetWorldVertices();
                if (verts.empty()) continue;

                float minX = 1e9f, maxX = -1e9f;
                float minY = 1e9f, maxY = -1e9f;
                for (const auto& v : verts) {
                    minX = std::min(minX, v.x);
                    maxX = std::max(maxX, v.x);
                    minY = std::min(minY, v.y);
                    maxY = std::max(maxY, v.y);
                }

                if (minX < 0.f) {
                    body.position.x += -minX;
                    body.velocity.x = std::abs(body.velocity.x) * body.restitution;
                }
                else if (maxX > screenWidth) {
                    body.position.x -= (maxX - screenWidth);
                    body.velocity.x = -std::abs(body.velocity.x) * body.restitution;
                }

                if (minY < 0.f) {
                    body.position.y += -minY;
                    body.velocity.y = std::abs(body.velocity.y) * body.restitution;
                }
                else if (maxY > screenHeight) {
                    body.position.y -= (maxY - screenHeight);
                    body.velocity.y = -std::abs(body.velocity.y) * body.restitution;
                }
            }
        }
        else if (currentScene == 3) {
            // 1. Integrate: apply wind + gravity to every spawned circle
            //    (circles don't rotate, so there's no UpdateRotation call here).
            for (auto& circle : circles) {
                circle.ApplyForce(windForce * circle.mass);
                circle.UpdatePhysics(dt, gravity);
            }

            // 2. Triangle-zone repulsion: any circle whose center has
            //    entered the static triangle gets pushed radially outward
            //    from the triangle's centroid - a strong outward force
            //    (2800 * mass) is applied for smooth acceleration, plus a
            //    small immediate positional nudge (2 units) to help it
            //    visibly escape the zone rather than lingering. If the
            //    circle's center lands exactly on the centroid (pushDir
            //    would be ~zero after normalizing), fall back to pushing
            //    straight up so it doesn't get stuck motionless. Circles
            //    are colored red while inside the triangle and green
            //    otherwise, giving immediate visual feedback.
            for (auto& circle : circles) {
                if (IsPointInTriangle(circle.position, triA, triB, triC)) {
                    sf::Vector2f pushDir = Math::Normalize(circle.position - triangleCentroid);
                    if (Math::LengthSq(pushDir) < 0.01f) {
                        pushDir = sf::Vector2f(0.f, -1.f);
                    }

                    circle.ApplyForce(pushDir * 2800.f * circle.mass);
                    circle.position += pushDir * 2.f;
                    circle.color = sf::Color::Red;
                }
                else {
                    circle.color = sf::Color(46, 204, 113);
                }
            }

            // 3. Clamp to screen bounds: standard circle-vs-AABB boundary
            //    clamp (position set exactly to the boundary, rather than
            //    offset like Scenes 1/2, since a circle's bounds are just
            //    position +/- radius) with restitution-scaled velocity reflection.
            for (auto& circle : circles) {
                float r = circle.radius;
                if (circle.position.x - r < 0.f) {
                    circle.position.x = r;
                    circle.velocity.x = std::abs(circle.velocity.x) * circle.restitution;
                }
                else if (circle.position.x + r > screenWidth) {
                    circle.position.x = screenWidth - r;
                    circle.velocity.x = -std::abs(circle.velocity.x) * circle.restitution;
                }

                if (circle.position.y - r < 0.f) {
                    circle.position.y = r;
                    circle.velocity.y = std::abs(circle.velocity.y) * circle.restitution;
                }
                else if (circle.position.y + r > screenHeight) {
                    circle.position.y = screenHeight - r;
                    circle.velocity.y = -std::abs(circle.velocity.y) * circle.restitution;
                }
            }
        }

        // --- Rendering ---
        window.clear(sf::Color(44, 62, 80));

        // HUD: current scene number, static control reminders, and a
        // scene-specific hint (only shown if the font loaded successfully).
        if (fontLoaded) {
            std::string hudString = "Scene: " + std::to_string(currentScene) + " | Controls: 1, 2, 3 (Switch Scene)\n";
            hudString += "Hold 'W' for Wind. ";
            if (currentScene == 3) {
                hudString += "Left Click to Spawn Circles.\nCircles inside triangle are pushed away.";
            }
            else {
                hudString += "\nPhysics simulation using Euler integration.";
            }
            uiText->setString(hudString);
            uiText->setPosition({ 15.f, 15.f });
            window.draw(*uiText);
        }

        // Draw only the shapes belonging to the currently active scene.
        if (currentScene == 1) {
            for (const auto& cap : capsules) {
                DrawCapsule(window, cap);
            }
        }
        else if (currentScene == 2) {
            for (const auto& body : convexShapes) {
                DrawConvex(window, body);
            }
        }
        else if (currentScene == 3) {
            // Draw the static target triangle...
            sf::ConvexShape targetTriangle;
            targetTriangle.setPointCount(3);
            targetTriangle.setPoint(0, triA);
            targetTriangle.setPoint(1, triB);
            targetTriangle.setPoint(2, triC);
            targetTriangle.setFillColor(sf::Color(52, 73, 94));
            targetTriangle.setOutlineColor(sf::Color::White);
            targetTriangle.setOutlineThickness(2.5f);
            window.draw(targetTriangle);

            // ...then every spawned circle, colored per its current state
            // (red/green, set during the update pass above).
            for (const auto& circle : circles) {
                sf::CircleShape cShape(circle.radius);
                cShape.setOrigin({ circle.radius, circle.radius });
                cShape.setPosition(circle.position);
                cShape.setFillColor(circle.color);
                cShape.setOutlineColor(sf::Color::White);
                cShape.setOutlineThickness(1.f);
                window.draw(cShape);
            }
        }

        window.display();
    }

    return 0;
}

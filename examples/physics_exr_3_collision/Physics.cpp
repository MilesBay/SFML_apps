#include "Physics.h"
#include <algorithm>

// --- Capsule Member Functions ---
sf::Vector2f Capsule::GetWorldA() const {
    sf::Vector2f dir(std::cos(angle), std::sin(angle));
    return position - dir * (length * 0.5f);
}

sf::Vector2f Capsule::GetWorldB() const {
    sf::Vector2f dir(std::cos(angle), std::sin(angle));
    return position + dir * (length * 0.5f);
}

void Capsule::UpdateRotation(float dt) {
    angle += angularSpeed * dt;
    if (angle > 2.f * Math::PI) angle -= 2.f * Math::PI;
}

// --- ConvexBody Member Functions ---
std::vector<sf::Vector2f> ConvexBody::GetWorldVertices() const {
    std::vector<sf::Vector2f> worldVerts;
    worldVerts.reserve(localVertices.size());
    for (const auto& v : localVertices) {
        worldVerts.push_back(position + Math::Rotate(v, angle));
    }
    return worldVerts;
}

void ConvexBody::UpdateRotation(float dt) {
    angle += angularSpeed * dt;
    if (angle > 2.f * Math::PI) angle -= 2.f * Math::PI;
}

// --- Collision Algorithms ---
sf::Vector2f ClosestPointOnSegment(const sf::Vector2f& X, const sf::Vector2f& Y, const sf::Vector2f& P) {
    sf::Vector2f segment = Y - X;
    float lenSq = Math::LengthSq(segment);
    if (lenSq < 0.0001f) return X;

    float t = Math::Dot(P - X, segment) / lenSq;
    t = std::max(0.f, std::min(1.f, t));
    return X + t * segment;
}

void ResolveCapsuleCollision(Capsule& capA, Capsule& capB) {
    sf::Vector2f A1 = capA.GetWorldA();
    sf::Vector2f A2 = capA.GetWorldB();
    sf::Vector2f B1 = capB.GetWorldA();
    sf::Vector2f B2 = capB.GetWorldB();

    float minDist = 1e9f;
    sf::Vector2f bestA, bestB;

    sf::Vector2f projB_A1 = ClosestPointOnSegment(B1, B2, A1);
    float d1 = Math::Distance(A1, projB_A1);
    if (d1 < minDist) { minDist = d1; bestA = A1; bestB = projB_A1; }

    sf::Vector2f projB_A2 = ClosestPointOnSegment(B1, B2, A2);
    float d2 = Math::Distance(A2, projB_A2);
    if (d2 < minDist) { minDist = d2; bestA = A2; bestB = projB_A2; }

    sf::Vector2f projA_B1 = ClosestPointOnSegment(A1, A2, B1);
    float d3 = Math::Distance(projA_B1, B1);
    if (d3 < minDist) { minDist = d3; bestA = projA_B1; bestB = B1; }

    sf::Vector2f projA_B2 = ClosestPointOnSegment(A1, A2, B2);
    float d4 = Math::Distance(projA_B2, B2);
    if (d4 < minDist) { minDist = d4; bestA = projA_B2; bestB = B2; }

    float combinedRadii = capA.radius + capB.radius;
    if (minDist < combinedRadii) {
        float penetration = combinedRadii - minDist;
        sf::Vector2f normal;
        if (minDist > 0.001f) {
            normal = (bestA - bestB) / minDist;
        }
        else {
            normal = sf::Vector2f(0.f, -1.f);
        }

        float k = 12000.f;
        sf::Vector2f pushForce = normal * (k * penetration);

        capA.ApplyForce(pushForce);
        capB.ApplyForce(-pushForce);

        float percent = 0.5f;
        sf::Vector2f correction = normal * (penetration / (capA.mass + capB.mass)) * percent;
        capA.position += correction * capA.mass;
        capB.position -= correction * capB.mass;
    }
}

SATResult CheckSAT(const std::vector<sf::Vector2f>& vertsA, const sf::Vector2f& centerA,
    const std::vector<sf::Vector2f>& vertsB, const sf::Vector2f& centerB) {
    SATResult result;
    result.collided = true;
    result.depth = 1e9f;

    auto checkAxes = [&](const std::vector<sf::Vector2f>& verts) -> bool {
        size_t count = verts.size();
        for (size_t i = 0; i < count; ++i) {
            sf::Vector2f p1 = verts[i];
            sf::Vector2f p2 = verts[(i + 1) % count];
            sf::Vector2f edge = p2 - p1;

            sf::Vector2f axis(-edge.y, edge.x);
            axis = Math::Normalize(axis);

            float minA = 1e9f, maxA = -1e9f;
            for (const auto& v : vertsA) {
                float proj = Math::Dot(v, axis);
                minA = std::min(minA, proj);
                maxA = std::max(maxA, proj);
            }

            float minB = 1e9f, maxB = -1e9f;
            for (const auto& v : vertsB) {
                float proj = Math::Dot(v, axis);
                minB = std::min(minB, proj);
                maxB = std::max(maxB, proj);
            }

            if (maxA < minB || maxB < minA) {
                return false;
            }

            float overlap = std::min(maxA, maxB) - std::max(minA, minB);
            if (overlap < result.depth) {
                result.depth = overlap;
                result.normal = axis;
            }
        }
        return true;
        };

    if (!checkAxes(vertsA) || !checkAxes(vertsB)) {
        SATResult empty;
        empty.collided = false;
        return empty;
    }

    if (Math::Dot(centerA - centerB, result.normal) < 0.f) {
        result.normal = -result.normal;
    }

    return result;
}

void ResolveConvexCollision(ConvexBody& bodyA, ConvexBody& bodyB) {
    auto vertsA = bodyA.GetWorldVertices();
    auto vertsB = bodyB.GetWorldVertices();

    SATResult sat = CheckSAT(vertsA, bodyA.position, vertsB, bodyB.position);
    if (sat.collided) {
        float k = 15000.f;
        sf::Vector2f pushForce = sat.normal * (k * sat.depth);

        bodyA.ApplyForce(pushForce);
        bodyB.ApplyForce(-pushForce);

        float percent = 0.5f;
        sf::Vector2f correction = sat.normal * (sat.depth / (bodyA.mass + bodyB.mass)) * percent;
        bodyA.position += correction * bodyA.mass;
        bodyB.position -= correction * bodyB.mass;
    }
}

float AngleBetween(const sf::Vector2f& v1, const sf::Vector2f& v2) {
    float len1 = Math::Length(v1);
    float len2 = Math::Length(v2);
    if (len1 < 0.001f || len2 < 0.001f) return 0.f;

    float ratio = Math::Dot(v1, v2) / (len1 * len2);
    ratio = std::max(-1.f, std::min(1.f, ratio));
    return std::acos(ratio);
}

bool IsPointInTriangle(const sf::Vector2f& P, const sf::Vector2f& A, const sf::Vector2f& B, const sf::Vector2f& C) {
    sf::Vector2f v1 = A - P;
    sf::Vector2f v2 = B - P;
    sf::Vector2f v3 = C - P;

    float angle1 = AngleBetween(v1, v2);
    float angle2 = AngleBetween(v2, v3);
    float angle3 = AngleBetween(v3, v1);

    float sum = angle1 + angle2 + angle3;
    return std::abs(sum - 2.f * Math::PI) < 0.05f;
}

// --- Drawing Helpers ---
void DrawCapsule(sf::RenderWindow& window, const Capsule& cap) {
    sf::Vector2f A = cap.GetWorldA();
    sf::Vector2f B = cap.GetWorldB();

    sf::RectangleShape rect(sf::Vector2f(cap.length, cap.radius * 2.f));
    rect.setOrigin({ cap.length * 0.5f, cap.radius });
    rect.setPosition(cap.position);
    rect.setRotation(sf::radians(cap.angle));
    rect.setFillColor(cap.color);
    window.draw(rect);

    sf::CircleShape endCircle(cap.radius);
    endCircle.setOrigin({ cap.radius, cap.radius });
    endCircle.setFillColor(cap.color);

    endCircle.setPosition(A);
    window.draw(endCircle);

    endCircle.setPosition(B);
    window.draw(endCircle);
}

void DrawConvex(sf::RenderWindow& window, const ConvexBody& body) {
    auto worldVerts = body.GetWorldVertices();
    sf::ConvexShape shape;
    shape.setPointCount(worldVerts.size());
    for (size_t i = 0; i < worldVerts.size(); ++i) {
        shape.setPoint(i, worldVerts[i]);
    }
    shape.setFillColor(body.color);
    shape.setOutlineColor(sf::Color(255, 255, 255, 150));
    shape.setOutlineThickness(1.5f);
    window.draw(shape);
}

#include "Physics.h"
#include <algorithm>

// --- Capsule Member Functions ---

// Computes the world-space position of end point "A": start from the
// capsule's center (`position`) and step half the capsule's length in the
// direction OPPOSITE to its facing direction (cos(angle), sin(angle)).
// Together, GetWorldA()/GetWorldB() give the two endpoints of the
// capsule's central line segment, which is the shape actually used for
// collision testing (the rounded caps are just that segment "thickened"
// by `radius`).
sf::Vector2f Capsule::GetWorldA() const {
    sf::Vector2f dir(std::cos(angle), std::sin(angle));
    return position - dir * (length * 0.5f);
}

// Computes the world-space position of end point "B": start from the
// capsule's center and step half the capsule's length in the direction the
// capsule is facing. See GetWorldA() for the counterpart.
sf::Vector2f Capsule::GetWorldB() const {
    sf::Vector2f dir(std::cos(angle), std::sin(angle));
    return position + dir * (length * 0.5f);
}

// Integrates the capsule's orientation forward by one timestep at its
// fixed `angularSpeed`, then wraps `angle` back into [0, 2*PI) once it
// exceeds a full turn (simple modulo-style wrap, not a general fmod - only
// correct because angle increases by a small amount each frame).
void Capsule::UpdateRotation(float dt) {
    angle += angularSpeed * dt;
    if (angle > 2.f * Math::PI) angle -= 2.f * Math::PI;
}

// --- ConvexBody Member Functions ---

// Transforms every local-space vertex into world space: rotate it about
// the local origin by the body's current `angle` (Math::Rotate), then
// translate by the body's world `position`. This is recomputed on demand
// every time it's needed (collision testing, drawing, bounding-box checks)
// rather than being cached, since `angle`/`position` can change every frame.
std::vector<sf::Vector2f> ConvexBody::GetWorldVertices() const {
    std::vector<sf::Vector2f> worldVerts;
    worldVerts.reserve(localVertices.size());
    for (const auto& v : localVertices) {
        worldVerts.push_back(position + Math::Rotate(v, angle));
    }
    return worldVerts;
}

// Integrates the polygon's orientation forward by one timestep at its
// fixed `angularSpeed`, wrapping `angle` back into [0, 2*PI). Identical
// logic to Capsule::UpdateRotation.
void ConvexBody::UpdateRotation(float dt) {
    angle += angularSpeed * dt;
    if (angle > 2.f * Math::PI) angle -= 2.f * Math::PI;
}

// --- Collision Algorithms ---

// Projects point P onto the infinite line through segment [X, Y], then
// clamps the projection parameter `t` to [0, 1] so the result stays on the
// segment itself (not the extended line). `t` is the normalized position
// along the segment: t=0 -> X, t=1 -> Y, computed via the standard
// vector-projection formula t = dot(P - X, segment) / |segment|^2.
// Degenerate case: if the segment is (near) zero-length, just return X to
// avoid dividing by ~0.
sf::Vector2f ClosestPointOnSegment(const sf::Vector2f& X, const sf::Vector2f& Y, const sf::Vector2f& P) {
    sf::Vector2f segment = Y - X;
    float lenSq = Math::LengthSq(segment);
    if (lenSq < 0.0001f) return X;

    float t = Math::Dot(P - X, segment) / lenSq;
    t = std::max(0.f, std::min(1.f, t));
    return X + t * segment;
}

// Detects and resolves a collision between two capsules.
//
// Detection: a capsule is really just a line segment "thickened" by its
// radius, so the shortest distance between two capsules equals the
// shortest distance between their two central segments. That segment-to-
// segment distance is approximated here by checking the four combinations
// of "one segment's endpoint projected onto the other segment" (A1->segB,
// A2->segB, B1->segA, B2->segA) and keeping whichever pairing gives the
// smallest distance. (This is a common approximation - it can slightly
// misjudge true minimum distance for certain crossing-segment configs, but
// is cheap and good enough for a real-time demo.)
//
// Resolution: if the closest distance is less than the sum of the two
// radii, the capsules are overlapping. A separation `normal` is built from
// the vector between the two closest points (falling back to straight up
// if they're coincident to avoid a zero-length normalize). Two things are
// then applied:
//   1. A spring-like "push force" proportional to penetration depth
//      (F = k * penetration) is applied as an actual physics force to each
//      body (opposite directions), which will smoothly accelerate them
//      apart over subsequent frames via normal integration.
//   2. An immediate positional correction is also applied directly to
//      `position` (not run through the force/integration system), split
//      between the two bodies in proportion to their mass, so heavier
//      capsules get shoved out less than lighter ones. This prevents
//      visible sustained overlap while the spring force is still
//      accelerating them apart.
void ResolveCapsuleCollision(Capsule& capA, Capsule& capB) {
    sf::Vector2f A1 = capA.GetWorldA();
    sf::Vector2f A2 = capA.GetWorldB();
    sf::Vector2f B1 = capB.GetWorldA();
    sf::Vector2f B2 = capB.GetWorldB();

    float minDist = 1e9f;
    sf::Vector2f bestA, bestB;

    // Candidate 1: A1's closest approach to segment B.
    sf::Vector2f projB_A1 = ClosestPointOnSegment(B1, B2, A1);
    float d1 = Math::Distance(A1, projB_A1);
    if (d1 < minDist) { minDist = d1; bestA = A1; bestB = projB_A1; }

    // Candidate 2: A2's closest approach to segment B.
    sf::Vector2f projB_A2 = ClosestPointOnSegment(B1, B2, A2);
    float d2 = Math::Distance(A2, projB_A2);
    if (d2 < minDist) { minDist = d2; bestA = A2; bestB = projB_A2; }

    // Candidate 3: B1's closest approach to segment A.
    sf::Vector2f projA_B1 = ClosestPointOnSegment(A1, A2, B1);
    float d3 = Math::Distance(projA_B1, B1);
    if (d3 < minDist) { minDist = d3; bestA = projA_B1; bestB = B1; }

    // Candidate 4: B2's closest approach to segment A.
    sf::Vector2f projA_B2 = ClosestPointOnSegment(A1, A2, B2);
    float d4 = Math::Distance(projA_B2, B2);
    if (d4 < minDist) { minDist = d4; bestA = projA_B2; bestB = B2; }

    // The best (bestA, bestB) pair across all four candidates is treated as
    // the true closest-points-between-segments approximation.
    float combinedRadii = capA.radius + capB.radius;
    if (minDist < combinedRadii) {
        float penetration = combinedRadii - minDist;
        sf::Vector2f normal;
        if (minDist > 0.001f) {
            // Points are distinct enough to safely derive a direction from them.
            normal = (bestA - bestB) / minDist;
        }
        else {
            // Closest points coincide (fully overlapping segments) - pick an
            // arbitrary but consistent separation direction (straight up)
            // rather than normalizing a zero vector.
            normal = sf::Vector2f(0.f, -1.f);
        }

        // Spring-style repulsion force: stronger the deeper the penetration.
        float k = 12000.f;
        sf::Vector2f pushForce = normal * (k * penetration);

        capA.ApplyForce(pushForce);
        capB.ApplyForce(-pushForce);

        // Direct positional correction (Baumgarte-style), split by mass so
        // the correction magnitude for each body is inversely related to
        // its own mass relative to the pair's total mass. `percent` softens
        // the correction (only resolve half the penetration per frame) to
        // avoid jittery overcorrection.
        float percent = 0.5f;
        sf::Vector2f correction = normal * (penetration / (capA.mass + capB.mass)) * percent;
        capA.position += correction * capA.mass;
        capB.position -= correction * capB.mass;
    }
}

// Separating Axis Theorem (SAT) overlap test between two convex polygons.
//
// SAT states that two convex shapes are NOT overlapping if and only if
// there exists at least one axis onto which their projections don't
// overlap. For polygons, it's sufficient to test the axes perpendicular to
// each edge of both polygons (the edge normals) - if no separating axis is
// found among all of them, the polygons must be overlapping.
//
// For each edge normal axis:
//   - project every vertex of both polygons onto the axis (via Dot) to get
//     each polygon's [min, max] interval on that axis
//   - if the intervals don't overlap (maxA < minB or maxB < minA), that
//     axis separates the polygons -> no collision, bail out immediately
//   - otherwise, track the axis with the SMALLEST overlap seen so far; that
//     overlap (`depth`) and axis (`normal`) together form the minimum
//     translation vector (MTV) - the smallest push needed to separate the
//     shapes along the axis that resists it least
//
// `checkAxes` is a local lambda run twice (once per polygon's edge set) so
// both polygons' normals are tested, per the standard SAT algorithm for
// polygon-polygon collision.
//
// If both passes complete without finding a separating axis, `result`
// holds a valid MTV, but its `normal` direction is arbitrary (it points
// along whichever edge happened to produce the minimum overlap, which
// could face either polygon) - so it's flipped if necessary to guarantee
// it points from B towards A (checked via the sign of the dot product with
// centerA - centerB), which is the convention callers rely on.
SATResult CheckSAT(const std::vector<sf::Vector2f>& vertsA, const sf::Vector2f& centerA,
    const std::vector<sf::Vector2f>& vertsB, const sf::Vector2f& centerB) {
    SATResult result;
    result.collided = true;
    result.depth = 1e9f;

    auto checkAxes = [&](const std::vector<sf::Vector2f>& verts) -> bool {
        size_t count = verts.size();
        for (size_t i = 0; i < count; ++i) {
            // Build the current edge (p1 -> p2), wrapping around to the
            // first vertex after the last one to close the polygon.
            sf::Vector2f p1 = verts[i];
            sf::Vector2f p2 = verts[(i + 1) % count];
            sf::Vector2f edge = p2 - p1;

            // The candidate separating axis is the edge's normal
            // (perpendicular vector), normalized to unit length so that
            // projected intervals/overlap depths are in consistent units.
            sf::Vector2f axis(-edge.y, edge.x);
            axis = Math::Normalize(axis);

            // Project all of polygon A's vertices onto the axis to find A's
            // [minA, maxA] interval.
            float minA = 1e9f, maxA = -1e9f;
            for (const auto& v : vertsA) {
                float proj = Math::Dot(v, axis);
                minA = std::min(minA, proj);
                maxA = std::max(maxA, proj);
            }

            // Same for polygon B's [minB, maxB] interval.
            float minB = 1e9f, maxB = -1e9f;
            for (const auto& v : vertsB) {
                float proj = Math::Dot(v, axis);
                minB = std::min(minB, proj);
                maxB = std::max(maxB, proj);
            }

            // Intervals don't overlap -> this axis separates the polygons,
            // so they cannot possibly be colliding. Early-out.
            if (maxA < minB || maxB < minA) {
                return false;
            }

            // Intervals overlap on this axis; remember it if it's the
            // shallowest overlap found so far (candidate for the MTV).
            float overlap = std::min(maxA, maxB) - std::max(minA, minB);
            if (overlap < result.depth) {
                result.depth = overlap;
                result.normal = axis;
            }
        }
        return true;
        };

    // Test both polygons' edge normals; if either finds a separating axis,
    // there's no collision.
    if (!checkAxes(vertsA) || !checkAxes(vertsB)) {
        SATResult empty;
        empty.collided = false;
        return empty;
    }

    // Ensure the resolved normal consistently points away from B, towards
    // A, regardless of which polygon's edge produced the minimum-overlap axis.
    if (Math::Dot(centerA - centerB, result.normal) < 0.f) {
        result.normal = -result.normal;
    }

    return result;
}

// Detects and resolves a collision between two convex polygon bodies,
// mirroring ResolveCapsuleCollision's two-part strategy but driven by the
// SAT-computed minimum translation vector instead of a segment distance:
//   1. Apply a spring-like push force (proportional to penetration depth)
//      to each body for smooth, physically-integrated separation.
//   2. Apply an immediate, mass-weighted positional correction to avoid
//      visible lingering overlap.
void ResolveConvexCollision(ConvexBody& bodyA, ConvexBody& bodyB) {
    auto vertsA = bodyA.GetWorldVertices();
    auto vertsB = bodyB.GetWorldVertices();

    SATResult sat = CheckSAT(vertsA, bodyA.position, vertsB, bodyB.position);
    if (sat.collided) {
        // Spring-style repulsion force along the MTV normal, scaled by
        // penetration depth.
        float k = 15000.f;
        sf::Vector2f pushForce = sat.normal * (k * sat.depth);

        bodyA.ApplyForce(pushForce);
        bodyB.ApplyForce(-pushForce);

        // Direct positional correction, split by mass (same rationale as
        // ResolveCapsuleCollision) and softened by `percent` to avoid jitter.
        float percent = 0.5f;
        sf::Vector2f correction = sat.normal * (sat.depth / (bodyA.mass + bodyB.mass)) * percent;
        bodyA.position += correction * bodyA.mass;
        bodyB.position -= correction * bodyB.mass;
    }
}

// Returns the unsigned angle between v1 and v2, in radians, via
// acos(dot(v1,v2) / (|v1|*|v2|)). Guards against zero-length inputs
// (returns 0 rather than dividing by zero) and clamps the ratio passed to
// acos() into its valid domain [-1, 1], since floating-point rounding can
// occasionally push the raw ratio just outside that range and make acos()
// return NaN.
float AngleBetween(const sf::Vector2f& v1, const sf::Vector2f& v2) {
    float len1 = Math::Length(v1);
    float len2 = Math::Length(v2);
    if (len1 < 0.001f || len2 < 0.001f) return 0.f;

    float ratio = Math::Dot(v1, v2) / (len1 * len2);
    ratio = std::max(-1.f, std::min(1.f, ratio));
    return std::acos(ratio);
}

// Point-in-triangle test using the "angle sum" method: for each pair of
// triangle vertices, measure the angle subtended at P between the
// vectors pointing from P to each vertex. If P is INSIDE the triangle, the
// three subtended angles (P->A to P->B, P->B to P->C, P->C to P->A)
// sum to exactly a full turn (2*PI). If P is OUTSIDE, the triangle only
// "covers" part of the view from P, so the angles sum to less than 2*PI.
// A small epsilon (0.05 rad) accounts for floating-point error in the
// trig/acos computations inside AngleBetween.
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

// Renders a Capsule as a rotated rectangle body (width = length, height =
// 2*radius, origin centered so it rotates/positions correctly about the
// capsule's center) plus two circles of radius `radius` drawn at the
// capsule's world-space endpoints to form the rounded end caps.
void DrawCapsule(sf::RenderWindow& window, const Capsule& cap) {
    sf::Vector2f A = cap.GetWorldA();
    sf::Vector2f B = cap.GetWorldB();

    // The "body" of the capsule: a rectangle spanning its full length and
    // twice its radius in height, centered and rotated to match the
    // capsule's current position/angle.
    sf::RectangleShape rect(sf::Vector2f(cap.length, cap.radius * 2.f));
    rect.setOrigin({ cap.length * 0.5f, cap.radius });
    rect.setPosition(cap.position);
    rect.setRotation(sf::radians(cap.angle));
    rect.setFillColor(cap.color);
    window.draw(rect);

    // The rounded end caps: a single reusable circle shape repositioned to
    // each endpoint in turn.
    sf::CircleShape endCircle(cap.radius);
    endCircle.setOrigin({ cap.radius, cap.radius });
    endCircle.setFillColor(cap.color);

    endCircle.setPosition(A);
    window.draw(endCircle);

    endCircle.setPosition(B);
    window.draw(endCircle);
}

// Renders a ConvexBody as an sf::ConvexShape built directly from its
// current world-space vertices (so it's rebuilt fresh every frame,
// reflecting the body's current position/rotation), with a translucent
// white outline for visual definition against similarly-colored shapes.
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

#pragma once

#include "MathHelpers.h"
#include <SFML/Graphics.hpp>
#include <vector>

// Base Physics Body
//
// Minimal rigid-body-style particle: tracks position/velocity/mass/
// restitution and integrates linear motion via simple (semi-implicit)
// Euler integration. Does NOT track rotation/angular velocity itself -
// derived shapes (Capsule, ConvexBody) add their own angle/angularSpeed
// and integrate rotation separately via UpdateRotation().
class PhysicsBody {
public:
    sf::Vector2f position;     // World-space position (origin/centroid of the shape).
    sf::Vector2f velocity;     // Current linear velocity (units/second).
    sf::Vector2f force;        // Accumulated force for the current frame; reset to zero after each UpdatePhysics() call.
    float mass;                // Mass used for F = m*a; also used to weight collision-correction splits (heavier bodies move less).
    float restitution;         // "Bounciness" factor in [0,1] used when resolving collisions with the world boundary (0 = no bounce, 1 = perfectly elastic).

    // Constructs a body at `pos` with mass `m` and restitution `rest`.
    // Mass is clamped to a small positive floor (0.001f) to avoid
    // division-by-zero when computing acceleration = force / mass.
    PhysicsBody(sf::Vector2f pos, float m, float rest)
        : position(pos), velocity(0.f, 0.f), force(0.f, 0.f), mass(m), restitution(rest) {
        if (mass < 0.001f) mass = 0.001f;
    }

    virtual ~PhysicsBody() {}

    // Accumulates an external force (e.g. gravity, wind, collision push-force)
    // into this frame's force total. Multiple calls in the same frame simply
    // sum together; the total is consumed and cleared by UpdatePhysics().
    void ApplyForce(const sf::Vector2f& f) {
        force += f;
    }

    // Advances the body's linear motion by one timestep `dt` using
    // semi-implicit (symplectic) Euler integration:
    //   1. acceleration = (accumulated force / mass) + gravity
    //   2. velocity is updated first using that acceleration
    //   3. position is then updated using the NEW velocity
    // This ordering (update velocity, then use it to update position) is
    // what makes it "semi-implicit" and gives noticeably more stable
    // results than naive/explicit Euler for spring- and impulse-like
    // forces (e.g. the collision push-forces used elsewhere in this file).
    // The accumulated force is cleared at the end so each frame starts fresh.
    void UpdatePhysics(float dt, const sf::Vector2f& gravity) {
        if (mass <= 0.f) return;

        sf::Vector2f acceleration = (force / mass) + gravity;
        velocity += acceleration * dt;
        position += velocity * dt;

        force = sf::Vector2f(0.f, 0.f);
    }
};

// Derived Shapes

// A "pill" shape: a line segment of length `length` swept by a circle of
// radius `radius`, centered at `position` and oriented by `angle`. Used
// because capsule-vs-capsule collision reduces to a simple
// closest-point-between-two-segments test plus a radius check (see
// ResolveCapsuleCollision in Physics.cpp), which is cheap and robust even
// for rotating, elongated shapes.
class Capsule : public PhysicsBody {
public:
    float length;         // Length of the capsule's central line segment (excludes the rounded end caps).
    float radius;         // Radius of the rounded end caps / the "thickness" of the capsule.
    float angle;          // Current orientation in radians.
    float angularSpeed;   // Constant angular velocity (radians/second) - these capsules spin at a fixed rate rather than via torque.
    sf::Color color;       // Fill color used for rendering.

    // `rot` is the initial angle (radians), `m`/`rest` are mass/restitution
    // forwarded to PhysicsBody. angularSpeed defaults to a fixed 0.2 rad/s
    // spin for every capsule.
    Capsule(sf::Vector2f pos, float len, float rad, float rot, float m, float rest, sf::Color col)
        : PhysicsBody(pos, m, rest), length(len), radius(rad), angle(rot), angularSpeed(0.2f), color(col) {
    }

    // Returns the world-space position of the capsule's first end point
    // (the end in the -direction along its current angle).
    sf::Vector2f GetWorldA() const;
    // Returns the world-space position of the capsule's second end point
    // (the end in the +direction along its current angle).
    sf::Vector2f GetWorldB() const;
    // Advances `angle` by `angularSpeed * dt`, wrapping back into [0, 2*PI)
    // once it exceeds a full turn.
    void UpdateRotation(float dt);
};

// A convex polygon body defined by a set of vertices in LOCAL space
// (relative to `position`, before rotation). World-space vertices are
// computed on demand by rotating each local vertex by `angle` and
// translating by `position`. Collision between two ConvexBody instances is
// resolved via the Separating Axis Theorem (SAT) - see CheckSAT /
// ResolveConvexCollision in Physics.cpp.
class ConvexBody : public PhysicsBody {
public:
    std::vector<sf::Vector2f> localVertices; // Polygon vertices in local (unrotated, origin-relative) space; must be wound consistently (used as edges in CheckSAT).
    float angle;          // Current orientation in radians.
    float angularSpeed;   // Constant angular velocity (radians/second), like Capsule.
    sf::Color color;       // Fill color used for rendering.

    // `verts` are copied as the polygon's local-space vertices; `rot` is the
    // initial angle. angularSpeed defaults to a fixed 0.25 rad/s spin.
    ConvexBody(sf::Vector2f pos, const std::vector<sf::Vector2f>& verts, float rot, float m, float rest, sf::Color col)
        : PhysicsBody(pos, m, rest), localVertices(verts), angle(rot), angularSpeed(0.25f), color(col) {
    }

    // Computes and returns this body's vertices transformed into world
    // space: each local vertex is rotated by `angle` (about the local
    // origin) and then offset by `position`.
    std::vector<sf::Vector2f> GetWorldVertices() const;
    // Advances `angle` by `angularSpeed * dt`, wrapping back into [0, 2*PI)
    // once it exceeds a full turn. Identical behavior to Capsule::UpdateRotation.
    void UpdateRotation(float dt);
};

// A simple circular body. Unlike Capsule/ConvexBody it has no rotation
// state (a circle looks the same at any angle), so it relies solely on
// PhysicsBody's linear motion. Used in Scene 3 as projectiles that get
// spawned by mouse clicks and pushed out of a triangular "danger zone".
class CircleBody : public PhysicsBody {
public:
    float radius;    // Radius of the circle, used for both rendering and boundary/point-in-triangle collision checks.
    sf::Color color;  // Fill color; changed dynamically at runtime to signal state (e.g. red while overlapping the triangle).

    CircleBody(sf::Vector2f pos, float r, float m, float rest, sf::Color col)
        : PhysicsBody(pos, m, rest), radius(r), color(col) {
    }
};

// Result of a Separating-Axis-Theorem overlap test between two convex
// polygons (see CheckSAT). Only meaningful when `collided` is true.
struct SATResult {
    bool collided;         // True if no separating axis was found, i.e. the polygons overlap.
    sf::Vector2f normal;    // Minimum-translation-vector direction: the axis of least overlap, pointing from body B towards body A.
    float depth;            // Minimum-translation-vector magnitude: how far the shapes overlap along `normal`.
};

// Function Declarations

// Finds the closest point to `P` that lies on the line segment [X, Y]
// (clamped to the segment, not the infinite line). Used as the core
// primitive for capsule-vs-capsule distance testing.
sf::Vector2f ClosestPointOnSegment(const sf::Vector2f& X, const sf::Vector2f& Y, const sf::Vector2f& P);

// Detects and resolves an overlap between two capsules by checking the
// distance between their central line segments against their combined
// radii, then applying a spring-like separating force plus a direct
// positional correction (split proportionally by mass) if they overlap.
void ResolveCapsuleCollision(Capsule& capA, Capsule& capB);

// Runs the Separating Axis Theorem against the edge normals of both
// polygons (`vertsA`/`vertsB`, already in world space) to determine
// whether they overlap, and if so, the minimum-translation vector
// (normal + depth) needed to push them apart. `centerA`/`centerB` are
// used only to orient the resulting normal to point away from B, towards A.
SATResult CheckSAT(const std::vector<sf::Vector2f>& vertsA, const sf::Vector2f& centerA,
    const std::vector<sf::Vector2f>& vertsB, const sf::Vector2f& centerB);

// Detects and resolves an overlap between two convex polygon bodies using
// CheckSAT, then applies a spring-like separating force plus a direct
// positional correction (split proportionally by mass), mirroring
// ResolveCapsuleCollision's approach but for polygons.
void ResolveConvexCollision(ConvexBody& bodyA, ConvexBody& bodyB);

// Returns the unsigned angle (in radians, range [0, PI]) between two
// vectors, computed via acos(dot(v1,v2) / (|v1||v2|)). Returns 0 if either
// vector is (near) zero length. The dot-product ratio is clamped to
// [-1, 1] to guard against acos() receiving an out-of-domain value due to
// floating-point rounding.
float AngleBetween(const sf::Vector2f& v1, const sf::Vector2f& v2);

// Tests whether point `P` lies inside triangle ABC using the "angle sum"
// method: P is inside the triangle if and only if the angles it subtends
// to each pair of triangle vertices sum to a full turn (2*PI); if P is
// outside, the subtended angles sum to something less. A small epsilon
// (0.05 rad) accounts for floating-point error.
bool IsPointInTriangle(const sf::Vector2f& P, const sf::Vector2f& A, const sf::Vector2f& B, const sf::Vector2f& C);

// Draws a Capsule as a rotated rectangle (the central "body") plus two
// circles (the rounded end caps) at its world-space endpoints.
void DrawCapsule(sf::RenderWindow& window, const Capsule& cap);
// Draws a ConvexBody as an sf::ConvexShape built from its world-space
// vertices, with a translucent white outline.
void DrawConvex(sf::RenderWindow& window, const ConvexBody& body);

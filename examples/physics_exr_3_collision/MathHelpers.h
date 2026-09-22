#pragma once

#include <SFML/Graphics.hpp>
#include <cmath>

// Small collection of free-standing 2D vector-math helpers built on top of
// sf::Vector2f. These are the low-level building blocks used throughout the
// physics/collision code (Physics.cpp, ShapeCollision.cpp) - everything from
// impulse resolution to SAT projection ultimately reduces to Dot/Cross/
// Normalize calls defined here.
namespace Math {
// Single-precision Pi, used for angle conversions and rotational math
// (e.g. converting between radians/degrees, or spinning shapes by a
// fraction of a full turn) elsewhere in the physics engine.
const float PI = 3.14159265359f;

// Dot product of two 2D vectors: a.x*b.x + a.y*b.y.
// Geometrically, a . b = |a||b|cos(theta), so this is used to:
//  - measure how much two vectors point in the same direction
//    (positive = acute angle, negative = obtuse, zero = perpendicular)
//  - project one vector onto another (e.g. splitting velocity into
//    normal/tangential components during collision resolution)
inline float Dot(const sf::Vector2f& a, const sf::Vector2f& b) {
    return a.x * b.x + a.y * b.y;
}

// 2D "cross product" (really the z-component of the 3D cross product with
// both inputs' z = 0): a.x*b.y - a.y*b.x.
// The result is a scalar, not a vector. Its sign tells you the rotational
// direction from a to b (positive = b is counter-clockwise from a, negative
// = clockwise), and its magnitude is |a||b|sin(theta) - i.e. twice the
// signed area of the triangle formed by a and b. Used for winding/
// orientation tests (e.g. point-in-polygon, edge-side tests) in SAT-based
// collision code.
inline float Cross(const sf::Vector2f& a, const sf::Vector2f& b) {
    return a.x * b.y - a.y * b.x;
}

// Squared length (magnitude) of a vector: v.x*v.x + v.y*v.y.
// Prefer this over Length() whenever only a *comparison* of magnitudes is
// needed (e.g. "is this distance less than radius?"), since it avoids the
// relatively expensive std::sqrt call.
inline float LengthSq(const sf::Vector2f& v) {
    return v.x * v.x + v.y * v.y;
}

// Actual length (magnitude) of a vector, i.e. sqrt(LengthSq(v)).
inline float Length(const sf::Vector2f& v) {
    return std::sqrt(LengthSq(v));
}

// Returns a unit-length vector pointing in the same direction as v.
// Guards against division-by-zero/NaN for (near-)zero-length vectors: if
// the length is below a small epsilon (0.0001f), the zero vector is
// returned instead of dividing by ~0. This matters a lot in physics code,
// where things like a zero relative-velocity vector are common and must
// not produce NaNs that poison the rest of the simulation.
inline sf::Vector2f Normalize(const sf::Vector2f& v) {
    float len = Length(v);
    if (len > 0.0001f) {
        return v / len;
    }
    return sf::Vector2f(0.f, 0.f);
}

// Euclidean distance between two points, computed as the length of the
// vector connecting them (a - b).
inline float Distance(const sf::Vector2f& a, const sf::Vector2f& b) {
    return Length(a - b);
}

// Rotates vector v counter-clockwise by angleRad radians about the origin,
// using the standard 2D rotation matrix:
//   [cosA  -sinA] [x]
//   [sinA   cosA] [y]
// Used for orienting shapes (e.g. rotating a convex body's local-space
// vertices/normals into world space based on its current rotation).
inline sf::Vector2f Rotate(const sf::Vector2f& v, float angleRad) {
    float cosA = std::cos(angleRad);
    float sinA = std::sin(angleRad);
    return sf::Vector2f(v.x * cosA - v.y * sinA, v.x * sinA + v.y * cosA);
}
}

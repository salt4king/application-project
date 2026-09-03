#pragma once

#include <cmath>

/* ---------------------------------------------------------------------------
 * Vec2 -- a minimal 2D vector.
 *
 * Why hand-roll this instead of using std::pair or two loose doubles?
 * Because every layer above this one (shapes, distance math, the visualizer)
 * talks in terms of points and directions. Giving that concept a name, and
 * giving it the operators you'd expect, means the geometry code below reads
 * like the math it implements instead of like index bookkeeping.
 *
 * Everything is constexpr/inline: this is a header-only value type, so the
 * compiler flattens it away entirely. There is no runtime cost to the
 * abstraction.
 * ------------------------------------------------------------------------- */
struct Vec2 {
    double x = 0.0;
    double y = 0.0;

    constexpr Vec2() = default;
    constexpr Vec2(double x_, double y_) : x(x_), y(y_) {}
};

constexpr Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
constexpr Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
constexpr Vec2 operator*(Vec2 v, double s) { return {v.x * s, v.y * s}; }
constexpr Vec2 operator*(double s, Vec2 v) { return {v.x * s, v.y * s}; }
constexpr Vec2 operator-(Vec2 v) { return {-v.x, -v.y}; }

/* Dot product: projection of a onto b, scaled by |b|. Used constantly below
 * for "how far along this edge is the closest point?" queries. */
constexpr double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }

/* 2D cross product -- the z-component of the 3D cross product of (a,0)x(b,0).
 * Its SIGN tells us which side of a the vector b lies on, which is exactly
 * what the point-in-convex-polygon test needs. */
constexpr double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }

/* Squared length. Preferred over length() wherever we only need to COMPARE
 * magnitudes, because it avoids a sqrt and, more importantly, avoids the
 * rounding that sqrt introduces. */
constexpr double lengthSquared(Vec2 v) { return dot(v, v); }

inline double length(Vec2 v) { return std::sqrt(lengthSquared(v)); }

inline double distance(Vec2 a, Vec2 b) { return length(a - b); }

/* Rotate a vector by `radians` counter-clockwise about the origin.
 * This is what lets a RectangularRobot carry a heading. */
inline Vec2 rotate(Vec2 v, double radians) {
    const double c = std::cos(radians);
    const double s = std::sin(radians);
    return {v.x * c - v.y * s, v.x * s + v.y * c};
}

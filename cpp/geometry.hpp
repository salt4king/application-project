#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

/* geometry.hpp -- vector math and the collision kernel.
 *
 * THE IDEA: every chassis is a convex CORE (a few points) plus a SKIN (a
 * thickness). A circle is one point with a thick skin; a rectangle is four
 * corners with no skin. Collision then reduces to one inequality:
 *
 *      coreDistance(a, b)  <=  skinA + skinB
 *
 * which works because inflating a shape by r pushes its boundary out by r in
 * every direction. One code path covers every shape pair, and a new chassis
 * costs one class and zero edits elsewhere. Full rationale in NOTES.md.
 */

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
    constexpr Vec2() = default;
    constexpr Vec2(double x_, double y_) : x(x_), y(y_) {}
};

constexpr Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
constexpr Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
constexpr Vec2 operator*(Vec2 v, double s) { return {v.x * s, v.y * s}; }

// "how far along" -- used to find the closest point on an edge.
constexpr double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }

// "which side" -- only the SIGN is ever used. Powers both overlap tests.
constexpr double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }

constexpr double lengthSquared(Vec2 v) { return dot(v, v); }
inline double distance(Vec2 a, Vec2 b) { return std::sqrt(lengthSquared(a - b)); }

inline Vec2 rotate(Vec2 v, double radians) {
    const double c = std::cos(radians), s = std::sin(radians);
    return {v.x * c - v.y * s, v.x * s + v.y * c};
}

namespace geom {

// Detects zero-length edges and "exactly on the boundary". Compared against
// squared lengths, hence the small value. The CONTACT tolerance is separate
// and lives in robots.hpp, since that one is policy rather than numerics.
constexpr double kDegenerateEpsilon = 1e-12;

/* A convex hull of up to 8 points. Fixed array, not a vector: collision runs
 * in a robot's control loop and allocating per query would hurt.
 *
 * The (i+1) % n below matters more than it looks. For a 1-vertex core (a
 * circle) it yields the degenerate edge (v0, v0), so circles need no special
 * case anywhere -- they travel the same path as rectangles. */
struct ConvexCore {
    static constexpr int kMaxVertices = 8;
    Vec2 v[kMaxVertices];
    int  n = 0;

    void add(Vec2 p) { if (n < kMaxVertices) v[n++] = p; }

    Vec2 edgeStart(int i) const { return v[i]; }
    Vec2 edgeEnd(int i)   const { return v[(i + 1) % n]; }
    int  edgeCount()      const { return n; }
};

inline ConvexCore makePoint(Vec2 p) {
    ConvexCore c;
    c.add(p);
    return c;
}

// Project p onto the segment, clamp to the ends, measure. The clamp is the
// point: the infinite line is not the segment. The zero-length guard is what
// lets a circle's 1-point core work.
inline double pointSegmentDistance(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2   ab      = b - a;
    const double lenSqrd = lengthSquared(ab);
    if (lenSqrd <= kDegenerateEpsilon) return distance(p, a);

    const double t = std::clamp(dot(p - a, ab) / lenSqrd, 0.0, 1.0);
    return distance(p, a + ab * t);
}

// Two segments cross when each straddles the other's line. Both halves are
// needed -- one alone only means they'd meet if extended.
//
// Strict on purpose: touching or collinear segments report false. In every
// such case a vertex lands on the other's boundary and contains() catches it,
// which avoids the nasty collinear special cases.
inline bool segmentsCross(Vec2 p1, Vec2 q1, Vec2 p2, Vec2 q2) {
    const Vec2 d1 = q1 - p1;
    const Vec2 d2 = q2 - p2;

    const double a1 = cross(d1, p2 - p1);   // which side of seg 1 is p2 on
    const double a2 = cross(d1, q2 - p1);
    const double b1 = cross(d2, p1 - p2);   // which side of seg 2 is p1 on
    const double b2 = cross(d2, q1 - p2);

    return ((a1 > 0.0) != (a2 > 0.0)) && ((b1 > 0.0) != (b2 > 0.0));
}

// Walk the edges and ask which side p is on. In a convex shape an interior
// point is on the same side of every edge.
//
// Accepts all-positive OR all-negative so clockwise input can't silently
// invert the answer. Fewer than 3 vertices encloses no area; a point lying on
// such a core is already distance 0 via pointSegmentDistance.
inline bool contains(const ConvexCore& core, Vec2 p) {
    if (core.n < 3) return false;

    bool anyPositive = false;
    bool anyNegative = false;

    for (int i = 0; i < core.n; ++i) {
        const Vec2 edge = core.edgeEnd(i) - core.edgeStart(i);
        const double side = cross(edge, p - core.edgeStart(i));
        if (side >  kDegenerateEpsilon) anyPositive = true;
        if (side < -kDegenerateEpsilon) anyNegative = true;
    }
    // On the boundary trips neither flag, so it counts as inside: touching is
    // contact.
    return !(anyPositive && anyNegative);
}

/* Distance between two cores; 0 if they overlap.
 *
 *   1. containment -- one swallows the other. Step 3 CANNOT see this: a speck
 *      inside a big box is far from every edge, so it would report a large gap.
 *      One vertex suffices; if boundaries don't cross, all or none are inside.
 *   2. crossing edges -- partial overlap, including a plus-sign where neither
 *      shape has a corner inside the other.
 *   3. apart -- for disjoint convex shapes the closest pair always has an end
 *      at a vertex, so corner-against-edge (both ways) finds it exactly.
 *
 * Steps 1 and 2 are a complete overlap test: those are the only two ways
 * convex shapes can overlap.
 */
inline double coreDistance(const ConvexCore& a, const ConvexCore& b) {
    if (a.n == 0 || b.n == 0) return std::numeric_limits<double>::infinity();

    if (contains(a, b.v[0]) || contains(b, a.v[0])) return 0.0;

    for (int i = 0; i < a.edgeCount(); ++i)
        for (int j = 0; j < b.edgeCount(); ++j)
            if (segmentsCross(a.edgeStart(i), a.edgeEnd(i),
                              b.edgeStart(j), b.edgeEnd(j)))
                return 0.0;

    double best = std::numeric_limits<double>::infinity();
    for (int i = 0; i < a.n; ++i)
        for (int j = 0; j < b.edgeCount(); ++j)
            best = std::min(best, pointSegmentDistance(
                a.v[i], b.edgeStart(j), b.edgeEnd(j)));

    for (int i = 0; i < b.n; ++i)
        for (int j = 0; j < a.edgeCount(); ++j)
            best = std::min(best, pointSegmentDistance(
                b.v[i], a.edgeStart(j), a.edgeEnd(j)));

    return best;
}

} // namespace geom

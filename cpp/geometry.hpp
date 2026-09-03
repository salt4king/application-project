#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

/* ===========================================================================
 * geometry.hpp -- 2D vector math, and the collision kernel.
 *
 * This file is the "how do I measure it" half of the solution. The other half,
 * robots.hpp, is the "what am I measuring" half. That split is deliberate: the
 * project's hint warns against putting the collision math inside
 * isColliding(), and keeping the math in its own file makes it structurally
 * impossible to drift back there.
 *
 *
 * THE CENTRAL IDEA OF THIS SOLUTION
 * ---------------------------------
 * The obvious way to solve this problem is one routine per pair of shape
 * types: circle-vs-circle, circle-vs-rectangle, rectangle-vs-rectangle. That
 * works, but it scales badly: N shape types needs N*(N+1)/2 routines, and
 * every new chassis you invent forces you to revisit every existing shape.
 *
 * Instead, notice that:
 *
 *      a circle    is a POINT      inflated by its radius
 *      a rectangle is FOUR CORNERS inflated by zero
 *
 * Both are "some convex set of points, grown outward by some radius". That is
 * a rich enough model to describe a circle, a rectangle (rotated or not), a
 * capsule, a rounded-corner bumper, or any convex chassis at all.
 *
 * Once every robot is described that way, collision stops being a case
 * analysis and becomes a single inequality:
 *
 *      distance(coreA, coreB)  <=  skinA + skinB
 *
 * Why that works: inflating a shape by r pushes its boundary outward by
 * exactly r in every direction. So two inflated shapes touch precisely when
 * the gap between their cores has been used up by the two skins.
 *
 * That inequality is the WHOLE collision system. Everything below exists to
 * compute its left-hand side, and that reduction is what lets isColliding()
 * be a one-liner.
 *
 *
 * SECTION 1: Vec2         -- 2D vector primitives
 * SECTION 2: namespace geom -- ConvexCore and the three distance primitives
 * ========================================================================= */


/* ===========================================================================
 * SECTION 1 -- Vec2
 *
 * Hand-rolled rather than std::pair or two loose doubles, because everything
 * above this point talks in terms of points and directions. Giving that
 * concept a name, with the operators you would expect, means the geometry
 * reads like the vector algebra it implements instead of like index
 * bookkeeping. All constexpr/inline, so the compiler flattens it away
 * entirely -- the abstraction costs nothing at runtime.
 * ========================================================================= */
struct Vec2 {
    double x = 0.0;
    double y = 0.0;

    constexpr Vec2() = default;
    constexpr Vec2(double x_, double y_) : x(x_), y(y_) {}
};

constexpr Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
constexpr Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
constexpr Vec2 operator*(Vec2 v, double s) { return {v.x * s, v.y * s}; }

/* Dot product: how far along b the vector a reaches, scaled by |b|. Used below
 * for "where on this edge is the closest point?" queries. */
constexpr double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }

/* 2D cross product -- the z-component of the 3D cross product of (a,0)x(b,0).
 * In 2D it comes out as a single number, and its SIGN tells us which side of a
 * the vector b lies on. Both segmentsCross() and contains() are built on
 * nothing but that fact. */
constexpr double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }

/* Squared length. Preferred wherever we only need to COMPARE magnitudes or
 * divide by |v|^2, since it avoids a sqrt and the rounding sqrt introduces. */
constexpr double lengthSquared(Vec2 v) { return dot(v, v); }

inline double distance(Vec2 a, Vec2 b) { return std::sqrt(lengthSquared(a - b)); }

/* Rotate counter-clockwise about the origin. This is the one function that
 * lets a RectangularRobot carry a heading. */
inline Vec2 rotate(Vec2 v, double radians) {
    const double c = std::cos(radians);
    const double s = std::sin(radians);
    return {v.x * c - v.y * s, v.x * s + v.y * c};
}


/* ===========================================================================
 * SECTION 2 -- namespace geom, the collision kernel.
 *
 * Only three primitives, each a few lines of plain vector algebra built from
 * nothing but dot and cross products:
 *
 *      pointSegmentDistance -- project a point onto a segment, clamp, measure
 *      segmentsCross        -- do two segments straddle each other?
 *      contains             -- is a point inside a convex polygon?
 *
 * coreDistance() composes those three into a general convex-polygon distance.
 * I chose this decomposition over the textbook segment-to-segment distance
 * routine (which solves a constrained minimisation over the unit square)
 * because these three are each derivable on a whiteboard, and I would rather
 * ship something I can explain than something I can only cite.
 * ========================================================================= */
namespace geom {

/* Tolerance used only to detect genuinely degenerate geometry (a zero-length
 * edge) and to treat "exactly on the boundary" as inside. Compared against
 * squared lengths and cross products, hence the very small value. This is NOT
 * the collision contact tolerance -- that lives in robots.hpp, because it is a
 * policy decision rather than a numerical one. */
constexpr double kDegenerateEpsilon = 1e-12;

/* ---------------------------------------------------------------------------
 * ConvexCore -- the convex hull of a handful of points, in world space.
 *
 * A fixed-capacity value type rather than a std::vector: a collision check
 * happens in the inner loop of a robot's control cycle, and heap-allocating a
 * vector per query per frame is exactly the kind of thing that quietly wrecks
 * real-time performance. Eight vertices covers any chassis you would actually
 * bolt together (up to an octagon).
 * ------------------------------------------------------------------------- */
struct ConvexCore {
    static constexpr int kMaxVertices = 8;

    Vec2 v[kMaxVertices];
    int  n = 0;

    void add(Vec2 p) {
        if (n < kMaxVertices) v[n++] = p;
    }

    /* Edge i runs from vertex i to vertex (i+1) mod n.
     *
     * The modular wrap does something quietly important. For a 1-vertex core
     * (a circle) it yields the DEGENERATE edge (v0, v0), and for a 2-vertex
     * core it yields the same segment twice. So the routines below never need
     * to ask "is this shape really just a point?" -- the degenerate edge falls
     * out of the general formula, and circle-vs-circle ends up travelling the
     * same code path as rectangle-vs-rectangle. */
    Vec2 edgeStart(int i) const { return v[i]; }
    Vec2 edgeEnd(int i)   const { return v[(i + 1) % n]; }
    int  edgeCount()      const { return n; }
};

inline ConvexCore makePoint(Vec2 p) {
    ConvexCore c;
    c.add(p);
    return c;
}

/* ---------------------------------------------------------------------------
 * PRIMITIVE 1: shortest distance from a point to a line segment.
 *
 * Three steps, all elementary:
 *
 *   1. Project p onto the infinite line through a and b. The projection sits
 *      at parameter t, where t = dot(p-a, ab) / dot(ab, ab). The numerator is
 *      how far along ab the point reaches; dividing by dot(ab,ab) = |ab|^2
 *      turns that into a fraction of the way along.
 *   2. Clamp t to [0, 1]. The infinite line is not the segment; if the
 *      projection falls off either end, the nearest point IS that end.
 *   3. Measure to whatever point we landed on.
 *
 * The zero-length guard is what makes circles work: a circle's core is a
 * single point, i.e. a segment with a == b, and then the answer is |p - a|.
 * ------------------------------------------------------------------------- */
inline double pointSegmentDistance(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2   ab      = b - a;
    const double lenSqrd = lengthSquared(ab);

    // Degenerate segment: it is really a point, so measure straight to it.
    if (lenSqrd <= kDegenerateEpsilon) return distance(p, a);

    const double t = std::clamp(dot(p - a, ab) / lenSqrd, 0.0, 1.0);
    return distance(p, a + ab * t);
}

/* ---------------------------------------------------------------------------
 * PRIMITIVE 2: do two segments cross?
 *
 * cross(d, q - origin) is positive on one side of the direction d and negative
 * on the other, so its sign answers "which side of this line is that point?".
 *
 * Two segments cross when each one straddles the other's line: p2 and q2 sit
 * on opposite sides of segment 1, AND p1 and q1 sit on opposite sides of
 * segment 2. Both conditions are needed -- one alone only says the segments
 * would cross if extended far enough.
 *
 * This is a STRICT test, so segments that merely touch, or lie along each
 * other, are reported as NOT crossing. That is deliberate, not an oversight:
 * in every such case a vertex of one shape lies on the other's boundary, and
 * contains() below already counts that as inside. Keeping the two tests
 * strictly separated avoids the fiddly collinear special cases that make the
 * textbook version of this routine so unpleasant.
 * ------------------------------------------------------------------------- */
inline bool segmentsCross(Vec2 p1, Vec2 q1, Vec2 p2, Vec2 q2) {
    const Vec2 d1 = q1 - p1;
    const Vec2 d2 = q2 - p2;

    const double a1 = cross(d1, p2 - p1);   // side of segment 1 that p2 is on
    const double a2 = cross(d1, q2 - p1);   // ... and q2
    const double b1 = cross(d2, p1 - p2);   // side of segment 2 that p1 is on
    const double b2 = cross(d2, q1 - p2);   // ... and q1

    return ((a1 > 0.0) != (a2 > 0.0)) && ((b1 > 0.0) != (b2 > 0.0));
}

/* ---------------------------------------------------------------------------
 * PRIMITIVE 3: is a point inside a convex core?
 *
 * Walk the edges and ask which side of each the point falls on, using the sign
 * of the cross product again. For a CONVEX polygon an interior point is on the
 * same side of every edge -- that is what convexity means here, and it is why
 * this is a three-line loop rather than a ray-casting routine.
 *
 * Written winding-order agnostic (it accepts "all non-negative" OR "all
 * non-positive") so a caller who hands us clockwise vertices gets a correct
 * answer instead of a silently inverted one.
 *
 * Cores with fewer than 3 vertices enclose no area, so nothing is inside them.
 * Returning false is not a cop-out: a point lying ON such a degenerate core is
 * already reported as distance 0 by primitive 1, so the case is covered --
 * just elsewhere.
 * ------------------------------------------------------------------------- */
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

    // A point exactly on the boundary trips neither flag and so counts as
    // inside, which is what we want here: touching is contact.
    return !(anyPositive && anyNegative);
}

/* ---------------------------------------------------------------------------
 * Shortest distance between two convex cores. Returns 0 if they overlap.
 *
 * Three questions, in order:
 *
 *   1. Does either core contain a vertex of the other? That catches total
 *      containment -- one robot swallowed by another, with their boundaries
 *      nowhere near each other. Step 3 cannot see this case at all, which is
 *      why the check is not redundant.
 *
 *      Testing a single vertex is enough: if the boundaries do not cross, then
 *      either every vertex of one core is inside the other or none is.
 *
 *   2. Do any of their edges cross? That catches partial overlap, including
 *      the awkward case of two rectangles crossing in a plus sign, where
 *      NEITHER shape has a corner inside the other yet they plainly overlap.
 *
 *   3. Otherwise they are disjoint, and the shortest distance between two
 *      disjoint convex polygons always has at least one endpoint at a vertex
 *      -- so checking every vertex against every edge of the other shape, in
 *      both directions, finds it exactly.
 *
 * Steps 1 and 2 together are a COMPLETE overlap test, because two convex
 * shapes can only overlap by containment or by crossing boundaries.
 *
 * Cost is O(|A| * |B|), at most 16 tests per stage for two rectangles. That is
 * nothing, and it is completely branch-free with respect to shape TYPE, which
 * is the entire point of the design.
 * ------------------------------------------------------------------------- */
inline double coreDistance(const ConvexCore& a, const ConvexCore& b) {
    if (a.n == 0 || b.n == 0) return std::numeric_limits<double>::infinity();

    // 1. Containment.
    if (contains(a, b.v[0]) || contains(b, a.v[0])) return 0.0;

    // 2. Crossing boundaries.
    for (int i = 0; i < a.edgeCount(); ++i) {
        for (int j = 0; j < b.edgeCount(); ++j) {
            if (segmentsCross(a.edgeStart(i), a.edgeEnd(i),
                              b.edgeStart(j), b.edgeEnd(j))) {
                return 0.0;
            }
        }
    }

    // 3. Disjoint: closest approach, vertex against edge, in both directions.
    double best = std::numeric_limits<double>::infinity();
    for (int i = 0; i < a.n; ++i) {
        for (int j = 0; j < b.edgeCount(); ++j) {
            best = std::min(best, pointSegmentDistance(
                a.v[i], b.edgeStart(j), b.edgeEnd(j)));
        }
    }
    for (int i = 0; i < b.n; ++i) {
        for (int j = 0; j < a.edgeCount(); ++j) {
            best = std::min(best, pointSegmentDistance(
                b.v[i], a.edgeStart(j), a.edgeEnd(j)));
        }
    }
    return best;
}

} // namespace geom

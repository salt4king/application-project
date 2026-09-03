#pragma once

#include "vec2.hpp"

#include <algorithm>
#include <limits>

/* ---------------------------------------------------------------------------
 * geom -- the geometry kernel.
 *
 * THE CENTRAL IDEA OF THIS SOLUTION
 * ---------------------------------
 * The obvious way to solve this problem is to write one routine per pair of
 * shape types: circle-vs-circle, circle-vs-rectangle, rectangle-vs-rectangle.
 * That works, but it scales badly: N shape types needs N*(N+1)/2 routines, and
 * every new chassis you invent forces you to revisit every existing shape.
 *
 * Instead, notice that:
 *
 *      a circle    is a POINT           inflated by its radius
 *      a rectangle is FOUR CORNERS      inflated by zero
 *
 * Both are "some convex set of points, grown outward by some radius". That is
 * a Minkowski sum of a convex polygon with a disc, and it is a rich enough
 * model to describe a circle, a rectangle (rotated or not), a capsule, a
 * rounded-corner bumper, or any convex chassis at all.
 *
 * Once every robot is described that way, collision stops being a case
 * analysis and becomes a single inequality:
 *
 *      distance(coreA, coreB)  <=  skinA + skinB
 *
 * Proof sketch: inflating a set by r pushes its boundary outward by exactly r
 * in every direction, so two inflated sets touch precisely when the gap
 * between their cores has been eaten up by the two skins.
 *
 * That inequality is the WHOLE collision system. Everything in this file
 * exists to compute its left-hand side, and the reduction is what lets
 * isColliding() stay a one-liner (see collision.hpp).
 * ------------------------------------------------------------------------- */
namespace geom {

/* Tolerance used only to detect genuinely degenerate geometry (a zero-length
 * edge). It is compared against SQUARED lengths, hence the very small value.
 * This is not the collision contact tolerance -- that lives in collision.hpp,
 * because it is a policy decision rather than a numerical one. */
constexpr double kDegenerateEpsilon = 1e-12;

/* ---------------------------------------------------------------------------
 * ConvexCore -- the convex hull of a handful of points, in world space.
 *
 * Deliberately a fixed-capacity value type rather than a std::vector: a
 * collision check happens in the inner loop of a robot's control cycle, and
 * heap-allocating a vector per query per frame is exactly the kind of thing
 * that quietly wrecks real-time performance. Eight vertices is plenty for any
 * chassis you'd actually bolt together (it covers up to an octagon).
 *
 * INVARIANT: vertices are stored in consistent winding order (this code does
 * not care whether it is clockwise or counter-clockwise -- see contains()).
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
     * The modular wrap is doing something subtle and important here. For a
     * 1-vertex core (a circle) it yields the single DEGENERATE edge (v0, v0),
     * and for a 2-vertex core (a capsule) it yields the same segment twice.
     * That means the distance routine below never needs to special-case
     * "this shape is really just a point" -- the degenerate edge falls out of
     * the general formula and the point-vs-point case is handled by the same
     * code path as polygon-vs-polygon. */
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
 * Shortest distance between two line SEGMENTS.
 *
 * This is the workhorse. It is Ericson's closest-point-between-segments
 * (Real-Time Collision Detection, 5.1.9), parameterising each segment as
 * P(s) = p1 + s*d1 and Q(t) = p2 + t*d2 with s,t clamped to [0,1], then
 * minimising |P(s) - Q(t)|.
 *
 * Two properties matter for us:
 *   1. It returns 0 when the segments intersect. That is what detects two
 *      rectangles overlapping in a "plus sign", where neither shape has a
 *      corner inside the other but their edges still cross.
 *   2. It stays correct when either or both segments have zero length. That
 *      is what makes circles work, since a circle's core is a single point.
 * ------------------------------------------------------------------------- */
inline double segmentSegmentDistance(Vec2 p1, Vec2 q1, Vec2 p2, Vec2 q2) {
    const Vec2 d1 = q1 - p1;   // direction and length of segment 1
    const Vec2 d2 = q2 - p2;   // direction and length of segment 2
    const Vec2 r  = p1 - p2;

    const double a = lengthSquared(d1);   // squared length of seg 1, always >= 0
    const double e = lengthSquared(d2);   // squared length of seg 2, always >= 0
    const double f = dot(d2, r);

    double s = 0.0;   // parameter along segment 1
    double t = 0.0;   // parameter along segment 2

    if (a <= kDegenerateEpsilon && e <= kDegenerateEpsilon) {
        // Both segments are points (e.g. circle vs circle). s = t = 0, and the
        // result below reduces to plain |p1 - p2|.
    } else if (a <= kDegenerateEpsilon) {
        // Segment 1 is a point: just project it onto segment 2.
        t = std::clamp(f / e, 0.0, 1.0);
    } else if (e <= kDegenerateEpsilon) {
        // Segment 2 is a point: project it onto segment 1.
        s = std::clamp(-dot(d1, r) / a, 0.0, 1.0);
    } else {
        // The general case: two honest segments.
        const double c     = dot(d1, r);
        const double b     = dot(d1, d2);
        const double denom = a * e - b * b;   // >= 0 by Cauchy-Schwarz

        // denom == 0 means the segments are parallel, so there is no unique
        // closest pair. Pinning s = 0 and letting the clamping below sort out
        // t picks a valid representative of the (tied) minimum.
        s = (denom > kDegenerateEpsilon)
                ? std::clamp((b * f - c * e) / denom, 0.0, 1.0)
                : 0.0;

        // Solve for t given s, then, if that lands outside the segment, clamp
        // it back onto the segment and re-solve for s. This two-step clamp is
        // what makes the routine correct for segments rather than infinite
        // lines.
        t = (b * s + f) / e;
        if (t < 0.0) {
            t = 0.0;
            s = std::clamp(-c / a, 0.0, 1.0);
        } else if (t > 1.0) {
            t = 1.0;
            s = std::clamp((b - c) / a, 0.0, 1.0);
        }
    }

    return distance(p1 + d1 * s, p2 + d2 * t);
}

/* ---------------------------------------------------------------------------
 * Is a point inside a convex core?
 *
 * Walk the edges and look at which side of each the point falls on, via the
 * sign of the 2D cross product. For a convex polygon, an interior point is on
 * the same side of every edge.
 *
 * Written to be winding-order agnostic (it accepts "all non-negative" OR
 * "all non-positive") so that a caller who hands us clockwise vertices gets a
 * correct answer instead of a silently inverted one. Robustness here is cheap;
 * a winding bug here would be invisible and awful to track down.
 *
 * Cores with fewer than 3 vertices enclose no area, so nothing is "inside"
 * them. Returning false is not a cop-out: a point lying ON such a degenerate
 * core is already reported as distance 0 by the segment routine above, so the
 * case is covered -- just elsewhere.
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

    // Points exactly on the boundary trip neither flag and so count as inside,
    // which is what we want: touching is contact.
    return !(anyPositive && anyNegative);
}

/* ---------------------------------------------------------------------------
 * Shortest distance between two convex cores. Returns 0 if they overlap.
 *
 * Two convex sets overlap in exactly one of two ways, and we need both:
 *
 *   (a) their boundaries cross -- caught by the edge-pair loop, since two
 *       crossing segments are at distance 0;
 *   (b) one is entirely inside the other, with boundaries never touching --
 *       missed entirely by the edge loop (all those edges are far apart!),
 *       which is why the containment test below is not redundant.
 *
 * Testing a single vertex per core is sufficient for case (b): if the
 * boundaries do not cross, then either every vertex of one is inside the other
 * or none is. And if the boundaries DO cross, we return 0 via the edge loop
 * regardless of what the containment test happened to say.
 *
 * Cost is O(|A| * |B|) segment tests -- at most 16 for two rectangles. For
 * chassis-sized inputs that is far cheaper than the branchier alternatives,
 * and it is completely branch-free with respect to shape TYPE, which is the
 * whole point.
 * ------------------------------------------------------------------------- */
inline double coreDistance(const ConvexCore& a, const ConvexCore& b) {
    if (a.n == 0 || b.n == 0) return std::numeric_limits<double>::infinity();

    // Case (b): total containment.
    if (contains(a, b.v[0]) || contains(b, a.v[0])) return 0.0;

    // Case (a) and the disjoint case: closest approach of the boundaries.
    double best = std::numeric_limits<double>::infinity();
    for (int i = 0; i < a.edgeCount(); ++i) {
        for (int j = 0; j < b.edgeCount(); ++j) {
            best = std::min(best, segmentSegmentDistance(
                a.edgeStart(i), a.edgeEnd(i),
                b.edgeStart(j), b.edgeEnd(j)));
            if (best <= 0.0) return 0.0;   // cannot do better; bail out early
        }
    }
    return best;
}

} // namespace geom
